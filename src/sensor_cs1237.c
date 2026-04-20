// CS1237 ADC sensor for probe/weight applications
//
// This file implements the CS1237 analog-to-digital converter interface
// used by the Artillery M1 probe/weigh system. It provides bulk ADC
// sampling via the Klipper sensor_bulk infrastructure and integrates
// with the trsync trigger system for probe homing.
//
// Copyright (C) 2024 - Gareth Farrington <gareth@waves.ky>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_*=
#include "board/gpio.h" // struct gpio_in, struct gpio_out
#include "board/irq.h" // irq_disable, irq_enable, irq_poll
#include "board/misc.h" // timer_read_time
#include "basecmd.h" // oid_alloc, oid_lookup, foreach_oid
#include "command.h" // DECL_COMMAND, sendf
#include "sched.h" // DECL_TASK, struct task_wake, sched_add_timer, sched_del_timer
#include "sensor_bulk.h" // struct sensor_bulk, sensor_bulk_report, sensor_bulk_reset
#include "trsync.h" // trsync_oid_lookup, trsync_do_trigger
#include "compiler.h" // container_of, ARRAY_SIZE

// CS1237 communication timing (in timer ticks)
#define CS1237_MIN_PULSE_NS 300

static uint32_t
cs1237_min_pulse_ticks(void)
{
    return timer_from_us(CS1237_MIN_PULSE_NS * 1000) / 1000000;
}

struct cs1237_sensor {
    struct timer timer;
    uint8_t pending_flag;
    uint32_t rest_ticks;
    uint32_t threshold;
    uint32_t last_adc;
    uint32_t trigger_clock;
    uint8_t trsync_oid;
    uint8_t trigger_reason;
    uint8_t error_reason;
    uint8_t homing_flag;
    struct gpio_in dout;
    struct gpio_out sclk;
    struct sensor_bulk sb;
};

static struct task_wake wake_cs1237;

/****************************************************************
 * Low-level bit-banging for CS1237
 ****************************************************************/

// Pause without IRQ (for clock high phase)
static void
cs1237_delay_noirq(uint32_t pulse)
{
#if CONFIG_MACH_AVR
    // Optimize avr
    asm("nop\n\tnop");
#else
    uint32_t end = timer_read_time() + pulse;
    while (timer_is_before(timer_read_time(), end))
        ;
#endif
}

// Pause with IRQ polling (for clock low phase)
static void
cs1237_delay(uint32_t pulse)
{
#if CONFIG_MACH_AVR
    // Optimize avr
    return;
#else
    uint32_t end = timer_read_time() + pulse;
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
#endif
}

// Generate a single CS1237 clock pulse on the sclk pin
static void
cs1237_one_clk(struct gpio_out sclk)
{
    uint32_t pulse = cs1237_min_pulse_ticks();
    irq_disable();
    gpio_out_reset(sclk, 0);
    irq_enable();
    cs1237_delay(pulse);
}

// Read 24 bits from CS1237 (MSB first)
// Returns a 32-bit unsigned value
static uint32_t
cs1237_read_adc_raw(struct gpio_in dout, struct gpio_out sclk)
{
    uint32_t data = 0;
    uint_fast8_t i;
    uint32_t pulse = cs1237_min_pulse_ticks();

    // Read 24 data bits (MSB first)
    for (i = 0; i < 24; i++) {
        irq_disable();
        gpio_out_reset(sclk, 0);
        irq_enable();
        cs1237_delay(pulse);

        irq_disable();
        gpio_out_reset(sclk, 1);
        irq_enable();
        cs1237_delay(pulse);

        data = (data << 1) | gpio_in_read(dout);
    }

    // Send 3 more clocks (padding/stop)
    for (i = 0; i < 3; i++)
        cs1237_one_clk(sclk);

    return data;
}

// Check if CS1237 data is ready (DOUT goes low)
static uint_fast8_t
cs1237_is_data_ready(struct cs1237_sensor *cs)
{
    return !gpio_in_read(cs->dout);
}

// Event handler that wakes wake_cs1237 periodically
static uint_fast8_t
cs1237_event(struct timer *timer)
{
    struct cs1237_sensor *cs = container_of(timer, struct cs1237_sensor, timer);
    uint32_t rest_ticks = cs->rest_ticks;

    if (cs->homing_flag && cs->trsync_oid) {
        // In homing mode - check if trigger condition is met
        if (cs1237_is_data_ready(cs)) {
            cs->pending_flag = 1;
            sched_wake_task(&wake_cs1237);
            rest_ticks *= 8;
        } else {
            rest_ticks *= 4;
        }
    } else if (cs->rest_ticks) {
        // Normal sampling mode
        if (cs1237_is_data_ready(cs)) {
            cs->pending_flag = 1;
            sched_wake_task(&wake_cs1237);
        }
        rest_ticks *= 4;
    }

    cs->timer.waketime += rest_ticks;
    return SF_RESCHEDULE;
}

static void
add_sample(struct cs1237_sensor *cs, uint8_t oid, uint32_t counts)
{
    cs->sb.data[cs->sb.data_count] = counts;
    cs->sb.data[cs->sb.data_count + 1] = counts >> 8;
    cs->sb.data[cs->sb.data_count + 2] = counts >> 16;
    cs->sb.data[cs->sb.data_count + 3] = counts >> 24;
    cs->sb.data_count += 4;

    if (cs->sb.data_count + 4 > ARRAY_SIZE(cs->sb.data))
        sensor_bulk_report(&cs->sb, oid);
}

// CS1237 ADC query - read and process one sample
static void
cs1237_read_adc(struct cs1237_sensor *cs, uint8_t oid)
{
    uint32_t start = timer_read_time();
    uint32_t adc = cs1237_read_adc_raw(cs->dout, cs->sclk);
    cs->pending_flag = 0;
    barrier();

    // Convert from 24-bit two's complement to 32-bit signed
    int32_t counts = (int32_t)(adc << 8);
    counts = counts >> 8;

    // Store last reading for threshold comparison
    cs->last_adc = counts;

    // Check homing threshold
    if (cs->homing_flag && cs->trsync_oid && cs->threshold > 0) {
        int32_t diff = counts - (int32_t)cs->trigger_clock;
        if (diff < 0) diff = -diff;
        if (diff >= cs->threshold) {
            struct trsync *ts = trsync_oid_lookup(cs->trsync_oid);
            if (ts) {
                trsync_do_trigger(ts, cs->trigger_reason);
            }
        }
    }

    // Add measurement to buffer
    add_sample(cs, oid, counts);
}

// Create a CS1237 sensor
void
command_config_cs1237(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_alloc(args[0], command_config_cs1237,
                                         sizeof(*cs));
    cs->timer.func = cs1237_event;
    cs->homing_flag = 0;
    cs->pending_flag = 0;
    cs->rest_ticks = 0;
    cs->threshold = 0;
    cs->last_adc = 0;
    cs->trigger_clock = 0;
    cs->trsync_oid = 0;
    cs->trigger_reason = 0;
    cs->error_reason = 0;
    cs->dout = gpio_in_setup(args[1], 1);
    cs->sclk = gpio_out_setup(args[2], 0);
    sensor_bulk_reset(&cs->sb);
}
DECL_COMMAND(command_config_cs1237, "config_cs1237 oid=%c dout_pin=%u"
             " sclk_pin=%u");

// Start/stop CS1237 sampling
void
command_query_cs1237(uint32_t *args)
{
    uint8_t oid = args[0];
    struct cs1237_sensor *cs = oid_lookup(oid, command_config_cs1237);
    sched_del_timer(&cs->timer);
    cs->pending_flag = 0;
    cs->rest_ticks = args[1];
    if (!cs->rest_ticks) {
        // End measurements
        gpio_out_write(cs->sclk, 0);
        return;
    }
    // Start new measurements
    sensor_bulk_reset(&cs->sb);
    irq_disable();
    cs->timer.waketime = timer_read_time() + cs->rest_ticks;
    sched_add_timer(&cs->timer);
    irq_enable();
}
DECL_COMMAND(command_query_cs1237, "query_cs1237 oid=%c rest_ticks=%u");

// Setup home trigger for CS1237
void
command_cs1237_setup_home(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    cs->trigger_clock = args[1];
    cs->threshold = args[2];
    cs->trsync_oid = args[3];
    cs->trigger_reason = args[4];
    cs->error_reason = args[5];
    cs->homing_flag = 1;
    if (args[1] == 0) {
        // Clear home state
        cs->homing_flag = 0;
        cs->trigger_clock = 0;
        cs->trsync_oid = 0;
    }
}
DECL_COMMAND(command_cs1237_setup_home,
             "cs1237_setup_home oid=%c clock=%u threshold=%u "
             "trsync_oid=%c trigger_reason=%c error_reason=%c");

// Query CS1237 home state
void
command_query_cs1237_home_state(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    sendf("cs1237_home_state oid=%c homing=%c trigger_clock=%u"
          , args[0], !!cs->homing_flag, cs->trigger_clock);
}
DECL_COMMAND(command_query_cs1237_home_state,
             "query_cs1237_home_state oid=%c");

// Read CS1237 data (reg read)
void
command_query_cs1237_read(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    uint8_t reg = (uint8_t)args[1];
    uint8_t len = (uint8_t)args[2];
    uint8_t buf[4];
    (void)reg;
    uint32_t adc = cs1237_read_adc_raw(cs->dout, cs->sclk);
    buf[0] = adc & 0xFF;
    buf[1] = (adc >> 8) & 0xFF;
    buf[2] = (adc >> 16) & 0xFF;
    buf[3] = (adc >> 24) & 0xFF;
    sendf("query_cs1237_data oid=%c data=%*s", args[0], len, buf);
}
DECL_COMMAND(command_query_cs1237_read,
             "query_cs1237_read oid=%c reg=%u read_len=%u");

// Query CS1237 begin (configure speed)
void
command_query_cs1237_begin(uint32_t *args)
{
    uint8_t oid = args[0];
    uint32_t config = args[1];
    (void)oid;
    (void)config;
    sendf("query_cs1237_begin_read oid=%c config=%u", oid, config);
}
DECL_COMMAND(command_query_cs1237_begin, "query_cs1237_begin oid=%c config=%u");

// Query CS1237 update (trigger zeroing)
void
command_query_cs1237_update(uint32_t *args)
{
    (void)args;
}
DECL_COMMAND(command_query_cs1237_update, "query_cs1237_zero oid=%c");

// Query CS1237 config read
void
command_query_cs1237_config_r(uint32_t *args)
{
    uint32_t config = 640; // Default: 640 sps
    sendf("query_cs1237_zero_config_read oid=%c config=%u", args[0], config);
}
DECL_COMMAND(command_query_cs1237_config_r, "query_cs1237_config_r oid=%c");

// Query CS1237 zero read only
void
command_query_cs1237_zero_read_only(uint32_t *args)
{
    struct cs1237_sensor *cs = oid_lookup(args[0], command_config_cs1237);
    uint8_t buf[4];
    uint32_t adc = cs1237_read_adc_raw(cs->dout, cs->sclk);
    buf[0] = adc & 0xFF;
    buf[1] = (adc >> 8) & 0xFF;
    buf[2] = (adc >> 16) & 0xFF;
    buf[3] = (adc >> 24) & 0xFF;
    sendf("query_cs1237_zero_read_o oid=%c data=%*s", args[0], 4, buf);
}
DECL_COMMAND(command_query_cs1237_zero_read_only,
             "query_cs1237_zero_read_only oid=%c");

// Background task that performs measurements
void
cs1237_capture_task(void)
{
    if (!sched_check_wake(&wake_cs1237))
        return;
    uint8_t oid;
    struct cs1237_sensor *cs;
    foreach_oid(oid, cs, command_config_cs1237) {
        if (cs->pending_flag)
            cs1237_read_adc(cs, oid);
    }
}
DECL_TASK(cs1237_capture_task);
