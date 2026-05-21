#include "signal_driver.h"

#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "ti_msp_dl_config.h"

typedef struct {
    uint16_t duration_ms;
    uint8_t  led_on;
    uint8_t  buzzer_on;
} signal_step_t;

typedef struct {
    const signal_step_t *steps;
    uint8_t             step_count;
} signal_pattern_t;

typedef struct {
    const signal_step_t *steps;
    uint8_t             step_count;
    uint8_t             step_index;
    uint16_t            remaining_ms;
    app_event_id_t      event_id;
} signal_state_t;

static const signal_step_t k_start_steps[] = {
    {80u, 1u, 1u},
    {80u, 0u, 0u},
    {80u, 1u, 1u},
    {80u, 0u, 0u},
};

static const signal_step_t k_pass_steps[] = {
    {80u, 1u, 1u},
    {40u, 0u, 0u},
};

static const signal_step_t k_stop_steps[] = {
    {350u, 1u, 1u},
    {450u, 1u, 0u},
    {120u, 0u, 0u},
};

static signal_state_t s_signal;

static void set_led(uint8_t on)
{
    if (on != 0u) {
        DL_GPIO_setPins(GPIO_LED_PIN_2_PORT, GPIO_LED_PIN_2_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_LED_PIN_2_PORT, GPIO_LED_PIN_2_PIN);
    }
}

static void set_buzzer(uint8_t on)
{
    if (on != 0u) {
        DL_GPIO_clearPins(GPIO_LED_PIN_BEEP_PORT, GPIO_LED_PIN_BEEP_PIN);
    } else {
        DL_GPIO_setPins(GPIO_LED_PIN_BEEP_PORT, GPIO_LED_PIN_BEEP_PIN);
    }
}

static void set_outputs(uint8_t led_on, uint8_t buzzer_on)
{
    set_led(led_on);
    set_buzzer(buzzer_on);
}

static uint8_t event_priority(app_event_id_t event_id)
{
    switch (event_id) {
    case APP_EVENT_STOP:
        return 3u;
    case APP_EVENT_START:
        return 2u;
    case APP_EVENT_PASS_A:
    case APP_EVENT_PASS_B:
    case APP_EVENT_PASS_C:
    case APP_EVENT_PASS_D:
        return 1u;
    case APP_EVENT_NONE:
    default:
        return 0u;
    }
}

static signal_pattern_t select_pattern(app_event_id_t event_id)
{
    signal_pattern_t pattern = {NULL, 0u};

    switch (event_id) {
    case APP_EVENT_START:
        pattern.steps = k_start_steps;
        pattern.step_count = (uint8_t)(sizeof(k_start_steps) / sizeof(k_start_steps[0]));
        break;
    case APP_EVENT_PASS_A:
    case APP_EVENT_PASS_B:
    case APP_EVENT_PASS_C:
    case APP_EVENT_PASS_D:
        pattern.steps = k_pass_steps;
        pattern.step_count = (uint8_t)(sizeof(k_pass_steps) / sizeof(k_pass_steps[0]));
        break;
    case APP_EVENT_STOP:
        pattern.steps = k_stop_steps;
        pattern.step_count = (uint8_t)(sizeof(k_stop_steps) / sizeof(k_stop_steps[0]));
        break;
    case APP_EVENT_NONE:
    default:
        break;
    }

    return pattern;
}

static void stop_pattern(void)
{
    s_signal.steps = NULL;
    s_signal.step_count = 0u;
    s_signal.step_index = 0u;
    s_signal.remaining_ms = 0u;
    s_signal.event_id = APP_EVENT_NONE;
    set_outputs(0u, 0u);
}

static void start_pattern(app_event_id_t event_id, signal_pattern_t pattern)
{
    s_signal.steps = pattern.steps;
    s_signal.step_count = pattern.step_count;
    s_signal.step_index = 0u;
    s_signal.remaining_ms = pattern.steps[0].duration_ms;
    s_signal.event_id = event_id;
    set_outputs(pattern.steps[0].led_on, pattern.steps[0].buzzer_on);
}

void signal_driver_init(void)
{
    stop_pattern();
}

void signal_emit(app_event_id_t event_id)
{
    signal_pattern_t pattern;

    if (event_id == APP_EVENT_NONE) {
        return;
    }

    pattern = select_pattern(event_id);
    if ((pattern.steps == NULL) || (pattern.step_count == 0u)) {
        return;
    }

    taskENTER_CRITICAL();
    if ((s_signal.event_id == APP_EVENT_NONE) ||
        (event_priority(event_id) >= event_priority(s_signal.event_id))) {
        start_pattern(event_id, pattern);
    }
    taskEXIT_CRITICAL();
}

void signal_driver_update(uint16_t elapsed_ms)
{
    taskENTER_CRITICAL();

    if ((s_signal.steps == NULL) || (s_signal.step_count == 0u)) {
        set_outputs(0u, 0u);
        taskEXIT_CRITICAL();
        return;
    }

    if (elapsed_ms >= s_signal.remaining_ms) {
        s_signal.step_index = (uint8_t)(s_signal.step_index + 1u);
        if (s_signal.step_index >= s_signal.step_count) {
            stop_pattern();
            taskEXIT_CRITICAL();
            return;
        }

        s_signal.remaining_ms = s_signal.steps[s_signal.step_index].duration_ms;
        set_outputs(s_signal.steps[s_signal.step_index].led_on,
                    s_signal.steps[s_signal.step_index].buzzer_on);
    } else {
        s_signal.remaining_ms = (uint16_t)(s_signal.remaining_ms - elapsed_ms);
    }

    taskEXIT_CRITICAL();
}
