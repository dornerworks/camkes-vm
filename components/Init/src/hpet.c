/*
 *  High Precision Event Timer emulation
 *
 *  Copyright (c) 2007 Alexander Graf
 *  Copyright (c) 2008 IBM Corporation
 *
 *  Authors: Beth Kon <bkon@us.ibm.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, see <http://www.gnu.org/licenses/>.
 *
 * *****************************************************************
 *
 * This driver attempts to emulate an HPET device in software.
 */

#include <autoconf.h>
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include <sel4/sel4.h>
#include <stdio.h>
#include <camkes.h>
#include "i8259.h"
#include "timers.h"
#include <platsupport/arch/tsc.h>

#include <vmm/mmio.h>
#include <vmm/vmm.h>

#define HPET_BASE               0xfed00000
#define HPET_LEN                0x500
#define HPET_CLK_PERIOD         10 /* 10 ns*/

#define FS_PER_NS               1000000       /* 1000000 femtoseconds == 1 ns */
#define HPET_MIN_TIMERS         3
#define HPET_MAX_TIMERS         3

#define HPET_NUM_IRQ_ROUTES     32

#define HPET_LEGACY_PIT_INT     0
#define HPET_LEGACY_RTC_INT     1

#define HPET_CFG_ENABLE 0x001
#define HPET_CFG_LEGACY 0x002

#define HPET_ID         0x000
#define HPET_PERIOD     0x004
#define HPET_CFG        0x010
#define HPET_STATUS     0x020
#define HPET_COUNTER    0x0f0
#define HPET_TN_CFG     0x000
#define HPET_TN_CMP     0x008
#define HPET_TN_ROUTE   0x010
#define HPET_CFG_WRITE_MASK  0x3

#define HPET_ID_NUM_TIM_SHIFT   8
#define HPET_ID_NUM_TIM_MASK    0x1f00

#define HPET_TN_TYPE_LEVEL       0x002
#define HPET_TN_ENABLE           0x004
#define HPET_TN_PERIODIC         0x008
#define HPET_TN_PERIODIC_CAP     0x010
#define HPET_TN_SIZE_CAP         0x020
#define HPET_TN_SETVAL           0x040
#define HPET_TN_32BIT            0x100
#define HPET_TN_INT_ROUTE_MASK  0x3e00
#define HPET_TN_FSB_ENABLE      0x4000
#define HPET_TN_FSB_CAP         0x8000
#define HPET_TN_CFG_WRITE_MASK  0x7f4e
#define HPET_TN_INT_ROUTE_SHIFT      9
#define HPET_TN_INT_ROUTE_CAP_SHIFT 32
#define HPET_TN_CFG_BITS_READONLY_OR_RESERVED 0xffff80b1U

#define RTC_ISA_IRQ 8

//#define HPET_DEBUG
#ifdef HPET_DEBUG
#define DPRINTF printf
#else
#define DPRINTF(...)
#endif

#define HPET_MSI_SUPPORT        0

#define HPET(obj) OBJECT_CHECK(HPETState, (obj), TYPE_HPET)

struct HPETState;
typedef struct HPETTimer {  /* timers */
    uint8_t tn;             /*timer number*/
    struct HPETState *state;
    /* Memory-mapped, software visible timer registers */
    uint64_t config;        /* configuration/cap */
    uint64_t cmp;           /* comparator */
    uint64_t fsb;           /* FSB route */
    /* Hidden register state */
    uint64_t period;        /* Last value written to comparator */
    uint8_t wrap_flag;      /* timer pop will indicate wrap for one-shot 32-bit
                             * mode. Next pop will be actual timer expiration.
                             */
    int tid;
} HPETTimer;

typedef struct HPETState {
    uint64_t hpet_offset;
    bool hpet_offset_saved;
    uint32_t flags;
    uint8_t rtc_irq_level;
    uint8_t num_timers;
    uint32_t intcap;
    HPETTimer timer[HPET_MAX_TIMERS];

    /* Memory-mapped, software visible registers */
    uint64_t capability;        /* capabilities */
    uint64_t config;            /* configuration */
    uint64_t isr;               /* interrupt status reg */
    uint64_t hpet_counter;      /* main counter */
    uint8_t  hpet_id;           /* instance id */
} HPETState;

static HPETState hpet_state;

static uint64_t tsc_frequency = 0;

static uint64_t current_time_ns() {
    return muldivu64(rdtsc_pure(), NS_IN_S, tsc_frequency);
}

static uint32_t hpet_in_legacy_mode(HPETState *s)
{
    return s->config & HPET_CFG_LEGACY;
}

static uint32_t timer_int_route(struct HPETTimer *timer)
{
    return (timer->config & HPET_TN_INT_ROUTE_MASK) >> HPET_TN_INT_ROUTE_SHIFT;
}

static uint32_t timer_fsb_route(HPETTimer *t)
{
    return t->config & HPET_TN_FSB_ENABLE;
}

static uint32_t hpet_enabled(HPETState *s)
{
    return s->config & HPET_CFG_ENABLE;
}

static uint32_t timer_is_periodic(HPETTimer *t)
{
    return t->config & HPET_TN_PERIODIC;
}

static uint32_t timer_enabled(HPETTimer *t)
{
    return t->config & HPET_TN_ENABLE;
}

static uint32_t hpet_time_after(uint64_t a, uint64_t b)
{
    return ((int32_t)(b - a) < 0);
}

static uint32_t hpet_time_after64(uint64_t a, uint64_t b)
{
    return ((int64_t)(b - a) < 0);
}

static uint64_t ticks_to_ns(uint64_t value)
{
    return value * HPET_CLK_PERIOD;
}

static uint64_t ns_to_ticks(uint64_t value)
{
    return value / HPET_CLK_PERIOD;
}

static uint64_t hpet_fixup_reg(uint64_t new, uint64_t old, uint64_t mask)
{
    new &= mask;
    new |= old & ~mask;
    return new;
}

static int activating_bit(uint64_t old, uint64_t new, uint64_t mask)
{
    return (!(old & mask) && (new & mask));
}

static int deactivating_bit(uint64_t old, uint64_t new, uint64_t mask)
{
    return ((old & mask) && !(new & mask));
}

static uint64_t hpet_get_ticks(HPETState *s)
{
    return ns_to_ticks(current_time_ns() + s->hpet_offset);
}

/*
 * calculate diff between comparator value and current ticks
 */
static inline uint64_t hpet_calculate_diff(HPETTimer *t, uint64_t current)
{

    if (t->config & HPET_TN_32BIT) {
        uint32_t diff, cmp;

        cmp = (uint32_t)t->cmp;
        diff = cmp - (uint32_t)current;
        diff = (int32_t)diff > 0 ? diff : (uint32_t)1;
        return (uint64_t)diff;
    } else {
        uint64_t diff, cmp;

        cmp = t->cmp;
        diff = cmp - current;
        diff = (int64_t)diff > 0 ? diff : (uint64_t)1;
        return diff;
    }
}

static void update_irq(struct HPETTimer *timer, int set)
{
    uint64_t mask;
    HPETState *s;
    int route;

    if (timer->tn <= 1 && hpet_in_legacy_mode(timer->state)) {
        /* if LegacyReplacementRoute bit is set, HPET specification requires
         * timer0 be routed to IRQ0 in NON-APIC or IRQ2 in the I/O APIC,
         * timer1 be routed to IRQ8 in NON-APIC or IRQ8 in the I/O APIC.
         */
        route = (timer->tn == 0) ? 0 : RTC_ISA_IRQ;
    } else {
        route = timer_int_route(timer);
    }
    s = timer->state;
    mask = 1 << timer->tn;
    if (!set || !timer_enabled(timer) || !hpet_enabled(timer->state)) {
        s->isr &= ~mask;
        if (!timer_fsb_route(timer)) {
            i8259_level_lower(route);
        }
    } else if (timer_fsb_route(timer)) {

    } else if (timer->config & HPET_TN_TYPE_LEVEL) {
        s->isr |= mask;
        i8259_level_raise(route);
    } else {
        s->isr &= ~mask;
        i8259_gen_irq(route);
    }
}

static int hpet_pre_save(void *opaque)
{
    HPETState *s = opaque;

    /* save current counter value */
    if (hpet_enabled(s)) {
        s->hpet_counter = hpet_get_ticks(s);
    }

    return 0;
}

static int hpet_pre_load(void *opaque)
{
    HPETState *s = opaque;

    /* version 1 only supports 3, later versions will load the actual value */
    s->num_timers = HPET_MIN_TIMERS;
    return 0;
}

static bool hpet_validate_num_timers(void *opaque, int version_id)
{
    HPETState *s = opaque;

    if (s->num_timers < HPET_MIN_TIMERS) {
        return false;
    } else if (s->num_timers > HPET_MAX_TIMERS) {
        return false;
    }
    return true;
}

static int hpet_post_load(void *opaque, int version_id)
{
    HPETState *s = opaque;

    /* Recalculate the offset between the main counter and guest time */
    if (!s->hpet_offset_saved) {
        s->hpet_offset = ticks_to_ns(s->hpet_counter) - current_time_ns();
    }

    /* Push number of timers into capability returned via HPET_ID */
    s->capability &= ~HPET_ID_NUM_TIM_MASK;
    s->capability |= (s->num_timers - 1) << HPET_ID_NUM_TIM_SHIFT;

    /* Derive HPET_MSI_SUPPORT from the capability of the first timer. */
    s->flags &= ~(1 << HPET_MSI_SUPPORT);
    if (s->timer[0].config & HPET_TN_FSB_CAP) {
        s->flags |= 1 << HPET_MSI_SUPPORT;
    }
    return 0;
}

static bool hpet_offset_needed(void *opaque)
{
    HPETState *s = opaque;

    return hpet_enabled(s) && s->hpet_offset_saved;
}

static bool hpet_rtc_irq_level_needed(void *opaque)
{
    HPETState *s = opaque;

    return s->rtc_irq_level != 0;
}

/*
 * timer expiration callback
 */
static void hpet_timer(void *opaque)
{
    HPETTimer *t = opaque;
    uint64_t diff;

    uint64_t period = t->period;
    uint64_t cur_tick = hpet_get_ticks(t->state);

    if (timer_is_periodic(t) && period != 0) {
        if (t->config & HPET_TN_32BIT) {
            while (hpet_time_after(cur_tick, t->cmp)) {
                t->cmp = (uint32_t)(t->cmp + t->period);
            }
        } else {
            while (hpet_time_after64(cur_tick, t->cmp)) {
                t->cmp += period;
            }
        }
        diff = hpet_calculate_diff(t, cur_tick);
        init_timer_oneshot_relative(t->tid, (int64_t)ticks_to_ns(diff));
    } else if (t->config & HPET_TN_32BIT && !timer_is_periodic(t)) {
        if (t->wrap_flag) {
            diff = hpet_calculate_diff(t, cur_tick);
            init_timer_oneshot_relative(t->tid, (int64_t)ticks_to_ns(diff));
            t->wrap_flag = 0;
        }
    }
    update_irq(t, 1);
}

static void hpet_set_timer(HPETTimer *t)
{
    uint64_t diff;
    uint32_t wrap_diff;  /* how many ticks until we wrap? */
    uint64_t cur_tick = hpet_get_ticks(t->state);

    /* whenever new timer is being set up, make sure wrap_flag is 0 */
    t->wrap_flag = 0;
    diff = hpet_calculate_diff(t, cur_tick);

    /* hpet spec says in one-shot 32-bit mode, generate an interrupt when
     * counter wraps in addition to an interrupt with comparator match.
     */
    if (t->config & HPET_TN_32BIT && !timer_is_periodic(t)) {
        wrap_diff = 0xffffffff - (uint32_t)cur_tick;
        if (wrap_diff < (uint32_t)diff) {
            diff = wrap_diff;
            t->wrap_flag = 1;
        }
    }
    init_timer_oneshot_relative(t->tid, (int64_t)ticks_to_ns(diff));
}

static void hpet_del_timer(HPETTimer *t)
{
    init_timer_stop(t->tid);
    update_irq(t, 0);
}

static void hpet_ram_read(vmm_vcpu_t *vcpu, void *opaque, uint32_t offset,
                          int size, seL4_Word *result)
{
    HPETState *s = opaque;
    uint64_t cur_tick, index;

    uintptr_t addr = HPET_BASE + offset;

    DPRINTF("Enter hpet_ram_readl at %" PRIx64 "\n", addr);

    index = offset;
    /*address range of all TN regs*/
    if (index >= 0x100 && index <= 0x3ff) {
        uint8_t timer_id = (addr - 0x100) / 0x20;
        HPETTimer *timer = &s->timer[timer_id];

        if (timer_id > s->num_timers) {
            DPRINTF("timer id out of range - %d\n", timer_id);
            *result = 0;
            return;
        }

        switch ((addr - 0x100) % 0x20) {
        case HPET_TN_CFG:
            *result = (seL4_Word)timer->config;
            break;
        case HPET_TN_CFG + 4: // Interrupt capabilities
            *result = (seL4_Word)(timer->config >> 32);
            break;
        case HPET_TN_CMP: // comparator register
            *result = (seL4_Word)timer->cmp;
            break;
        case HPET_TN_CMP + 4:
            *result = (seL4_Word)(timer->cmp >> 32);
            break;
        case HPET_TN_ROUTE:
            *result = (seL4_Word)timer->fsb;
            break;
        case HPET_TN_ROUTE + 4:
            *result = (seL4_Word)(timer->fsb >> 32);
            break;
        default:
            *result = 0;
            printf("invalid hpet_ram_readl - %lx\n", (seL4_Word)index);
            break;
        }
    } else {
        switch (index) {
        case HPET_ID:
            *result = (seL4_Word)s->capability;
            DPRINTF("hpet_id: %lx\n", *result);
            break;
        case HPET_PERIOD:
            *result = (seL4_Word)(s->capability >> 32);
            DPRINTF("hpet_period: %lx\n", *result);
            break;
        case HPET_CFG:
            *result = (seL4_Word)s->config;
            DPRINTF("hpet_cfg: %lx\n", *result);
            break;
        case HPET_CFG + 4:
            DPRINTF("invalid HPET_CFG + 4 hpet_ram_readl\n");
            *result = 0;
            break;
        case HPET_COUNTER:
            if (hpet_enabled(s)) {
                cur_tick = hpet_get_ticks(s);
            } else {
                cur_tick = s->hpet_counter;
            }
            DPRINTF("reading counter  = %lx\n", cur_tick);
            *result = (seL4_Word)cur_tick;
            break;
        case HPET_COUNTER + 4:
            if (hpet_enabled(s)) {
                cur_tick = hpet_get_ticks(s);
            } else {
                cur_tick = s->hpet_counter;
            }
            DPRINTF("reading counter + 4  = %lx\n", cur_tick);
            *result = (seL4_Word)(cur_tick >> 32);
            break;
        case HPET_STATUS:
            *result = (seL4_Word)s->isr;
            break;
        default:
            *result = 0;
            printf("invalid hpet_ram_readl - %lx\n", (seL4_Word)index);
            break;
        }
    }
}

static void hpet_ram_write(vmm_vcpu_t *vcpu, void *opaque, uint32_t offset,
                           int size, uint64_t value)
{
    int i;
    HPETState *s = opaque;
    uint64_t old_val, new_val, val, index;

    uintptr_t addr = HPET_BASE + offset;

    DPRINTF("Enter hpet_ram_writel at %lx = %lx\n", addr, value);
    index = offset;
    uint64_t res;
    hpet_ram_read(vcpu, opaque, offset, 4, &old_val);
    new_val = value;

    /*address range of all TN regs*/
    if (index >= 0x100 && index <= 0x3ff) {
        uint8_t timer_id = (addr - 0x100) / 0x20;
        DPRINTF("hpet_ram_writel timer_id = %#x\n", timer_id);
        if (timer_id > s->num_timers) {
            DPRINTF("timer id out of range\n");
            return;
        }
        HPETTimer *timer = &s->timer[timer_id];

        switch ((addr - 0x100) % 0x20) {
        case HPET_TN_CFG:
            DPRINTF("hpet_ram_writel HPET_TN_CFG\n");
            if (activating_bit(old_val, new_val, HPET_TN_FSB_ENABLE)) {
                update_irq(timer, 0);
            }
            val = hpet_fixup_reg(new_val, old_val, HPET_TN_CFG_WRITE_MASK);
            timer->config = (timer->config & 0xffffffff00000000ULL) | val;
            if (new_val & HPET_TN_32BIT) {
                timer->cmp = (uint32_t)timer->cmp;
                timer->period = (uint32_t)timer->period;
            }
            if (activating_bit(old_val, new_val, HPET_TN_ENABLE) &&
                hpet_enabled(s)) {
                hpet_set_timer(timer);
            } else if (deactivating_bit(old_val, new_val, HPET_TN_ENABLE)) {
                hpet_del_timer(timer);
            }
            break;
        case HPET_TN_CFG + 4: // Interrupt capabilities
            DPRINTF("invalid HPET_TN_CFG+4 write\n");
            break;
        case HPET_TN_CMP: // comparator register
            DPRINTF("hpet_ram_writel HPET_TN_CMP\n");
            if (timer->config & HPET_TN_32BIT) {
                new_val = (uint32_t)new_val;
            }
            if (!timer_is_periodic(timer)
                || (timer->config & HPET_TN_SETVAL)) {
                timer->cmp = (timer->cmp & 0xffffffff00000000ULL) | new_val;
            }
            if (timer_is_periodic(timer)) {
                /*
                 * FIXME: Clamp period to reasonable min value?
                 * Clamp period to reasonable max value
                 */
                new_val &= (timer->config & HPET_TN_32BIT ? ~0u : ~0ull) >> 1;
                timer->period =
                    (timer->period & 0xffffffff00000000ULL) | new_val;
            }
            timer->config &= ~HPET_TN_SETVAL;
            if (hpet_enabled(s)) {
                hpet_set_timer(timer);
            }
            break;
        case HPET_TN_CMP + 4: // comparator register high order
            DPRINTF("hpet_ram_writel HPET_TN_CMP + 4\n");
            if (!timer_is_periodic(timer)
                || (timer->config & HPET_TN_SETVAL)) {
                timer->cmp = (timer->cmp & 0xffffffffULL) | new_val << 32;
            } else {
                /*
                 * FIXME: Clamp period to reasonable min value?
                 * Clamp period to reasonable max value
                 */
                new_val &= (timer->config & HPET_TN_32BIT ? ~0u : ~0ull) >> 1;
                timer->period =
                    (timer->period & 0xffffffffULL) | new_val << 32;
                }
                timer->config &= ~HPET_TN_SETVAL;
                if (hpet_enabled(s)) {
                    hpet_set_timer(timer);
                }
                break;
        case HPET_TN_ROUTE:
            timer->fsb = (timer->fsb & 0xffffffff00000000ULL) | new_val;
            break;
        case HPET_TN_ROUTE + 4:
            timer->fsb = (new_val << 32) | (timer->fsb & 0xffffffff);
            break;
        default:
            printf("invalid hpet_ram_writel %lx <- %lx \n", addr, value);
            break;
        }
        return;
    } else {
        switch (index) {
        case HPET_ID:
            return;
        case HPET_CFG:
            val = hpet_fixup_reg(new_val, old_val, HPET_CFG_WRITE_MASK);
            s->config = (s->config & 0xffffffff00000000ULL) | val;
            if (activating_bit(old_val, new_val, HPET_CFG_ENABLE)) {
                /* Enable main counter and interrupt generation. */
                s->hpet_offset =
                    ticks_to_ns(s->hpet_counter) - current_time_ns();
                for (i = 0; i < s->num_timers; i++) {
                    if ((&s->timer[i])->cmp != ~0ULL) {
                        hpet_set_timer(&s->timer[i]);
                    }
                }
            } else if (deactivating_bit(old_val, new_val, HPET_CFG_ENABLE)) {
                /* Halt main counter and disable interrupt generation. */
                s->hpet_counter = hpet_get_ticks(s);
                for (i = 0; i < s->num_timers; i++) {
                    hpet_del_timer(&s->timer[i]);
                }
            }
            /* i8254 and RTC output pins are disabled
             * when HPET is in legacy mode */
            if (activating_bit(old_val, new_val, HPET_CFG_LEGACY)) {
                i8259_level_lower(0);
                i8259_level_lower(RTC_ISA_IRQ);
            } else if (deactivating_bit(old_val, new_val, HPET_CFG_LEGACY)) {
                i8259_level_lower(0);
                i8259_level_set(RTC_ISA_IRQ, s->rtc_irq_level);
            }
            break;
        case HPET_CFG + 4:
            DPRINTF("invalid HPET_CFG+4 write\n");
            break;
        case HPET_STATUS:
            val = new_val & s->isr;
            for (i = 0; i < s->num_timers; i++) {
                if (val & (1 << i)) {
                    update_irq(&s->timer[i], 0);
                }
            }
            break;
        case HPET_COUNTER:
            if (hpet_enabled(s)) {
                DPRINTF("Writing counter while HPET enabled!\n");
            }
            s->hpet_counter =
                (s->hpet_counter & 0xffffffff00000000ULL) | value;
            DPRINTF("HPET counter written. ctr = %#x -> %" PRIx64 "\n",
                    value, s->hpet_counter);
            break;
        case HPET_COUNTER + 4:
            if (hpet_enabled(s)) {
                DPRINTF("Writing counter while HPET enabled!\n");
            }
            s->hpet_counter =
                (s->hpet_counter & 0xffffffffULL) | (((uint64_t)value) << 32);
            DPRINTF("HPET counter + 4 written. ctr = %#x -> %" PRIx64 "\n",
                    value, s->hpet_counter);
            break;
        default:
            printf("invalid hpet_ram_writel %lx <- %lx \n", addr, value);
            break;
        }
    }
}

static void hpet_reset(HPETState *s)
{
    int i;

    for (i = 0; i < s->num_timers; i++) {
        HPETTimer *timer = &s->timer[i];

        hpet_del_timer(timer);
        timer->cmp = ~0ULL;
        timer->config = HPET_TN_PERIODIC_CAP | HPET_TN_SIZE_CAP;
        if (s->flags & (1 << HPET_MSI_SUPPORT)) {
            timer->config |= HPET_TN_FSB_CAP;
        }
        /* advertise availability of ioapic int */
        timer->config |=  (uint64_t)s->intcap << 32;
        timer->period = 0ULL;
        timer->wrap_flag = 0;
    }

    s->hpet_counter = 0ULL;
    s->hpet_offset = 0ULL;
    s->config = 0ULL;

    /* to document that the RTC lowers its output on reset as well */
    s->rtc_irq_level = 0;
}

static void hpet_handle_legacy_irq(void *opaque, int n, int level)
{
    HPETState *s = opaque;

    if (n == HPET_LEGACY_PIT_INT) {
        if (!hpet_in_legacy_mode(s)) {
            i8259_level_set(0, level);
        }
    } else {
        s->rtc_irq_level = level;
        if (!hpet_in_legacy_mode(s)) {
            i8259_level_set(RTC_ISA_IRQ, level);
        }
    }
}

static void hpet_realize(HPETState *s)
{
    HPETTimer *timer;

    s->hpet_id = 0;

    for (int i = 0; i < HPET_MAX_TIMERS; i++) {
        timer = &s->timer[i];
        timer->tid = TIMER_HPET0 + i;
        timer->tn = i;
        timer->state = s;
    }

    s->num_timers = HPET_MIN_TIMERS;

    /* 64-bit main counter; LegacyReplacementRoute. */
    s->capability = 0x8086a001ULL;
    s->capability |= (s->num_timers - 1) << HPET_ID_NUM_TIM_SHIFT;
    s->capability |= ((uint64_t)(HPET_CLK_PERIOD * FS_PER_NS) << 32);
}

void hpet_timer_interrupt(int completed) {
    for (int i = 0; i < HPET_MAX_TIMERS; i++) {
        HPETTimer *timer = &hpet_state.timer[i];
        if (completed & BIT(timer->tid)) {
            hpet_timer((void *)timer);
        }
    }
}

void hpet_reg_callbacks(vmm_vcpu_t *vcpu) {
    assert(NULL != vcpu);
    vmm_mmio_add_handler(&vcpu->mmio_list, HPET_BASE,
                         HPET_BASE + HPET_LEN,
                         &hpet_state, "HPET", hpet_ram_read, hpet_ram_write);
}

void hpet_pre_init(void) {
    tsc_frequency = init_timer_tsc_frequency();
    hpet_realize(&hpet_state);
    hpet_reset(&hpet_state);
}
