
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#define RED_OUTPUT_ENABLE_PIN   22  // wire from G22 to GND
#define GREEN_OUTPUT_SELECT_PIN 18  // wire from G18 to GND
#define DATA_LED                32
#define SIGNAL_LED              12
#define EXTRA_GND               25
#define DEBOUNCE_CD             250  // ms

const unsigned long a = 13 * 100;
const unsigned long b = 12 * 100;
const unsigned long c = 13 + 4;
const unsigned long d = 7 * 500;

#define DEBUG 0 // set 1 for slow, 0 for normal
// const unsigned long DEBUG_SLOWER = DEBUG ? 1000 : 1;
#ifdef CONFIG_APP_DEBUG_TIMING
    const unsigned long DEBUG_SLOWER = 1000UL;
#else
    const unsigned long DEBUG_SLOWER = 1UL;
#endif


const unsigned long T_SYNC_ON      = 50 * DEBUG_SLOWER;
const unsigned long BASE_PULSE_LEN = a * DEBUG_SLOWER;
const unsigned long PULSE_LOW_LEN  = b * DEBUG_SLOWER;
const unsigned long NUM_PULSES     = c;
const unsigned long PULSE_PAUSE_LEN = d * DEBUG_SLOWER;

const char* TAG = "main";

bool isEnabled = false;
bool isStateAlternative = false;
bool yield_wdg = true;

static inline long T_ON_n(uint8_t n)
{
    return (a + (n - 1) * 50) * DEBUG_SLOWER;
}

typedef enum {
    STATE_STOPPED,
    STATE_SYNC,
    STATE_PULSING,
    STATE_PAUSE
} pulse_state_t;

static volatile bool timerTriggered = false;
static pulse_state_t state = STATE_STOPPED;
static gptimer_handle_t gptimer = NULL;
static int dataPinState = 0;
static int signalPinState = 0;
static int dataPulseNum = 0;

void stopStateMachine(void);
void startStateMachine(void);
void stateMachine(void);

static TaskHandle_t stateTaskHandle = NULL;


static bool IRAM_ATTR timer_cb(gptimer_handle_t timer,
                               const gptimer_alarm_event_data_t *edata,
                               void *user_data)
{
    BaseType_t higherPriorityWoken = pdFALSE;
    vTaskNotifyGiveFromISR(stateTaskHandle, &higherPriorityWoken);
    return (higherPriorityWoken == pdTRUE);
}

void stateMachine(void)
{
    uint64_t next_alarm = 0;

    switch (state) {
        case STATE_STOPPED:
            gptimer_stop(gptimer);
            dataPinState = 0;
            gpio_set_level(DATA_LED, dataPinState);
            signalPinState = 0;
            gpio_set_level(SIGNAL_LED, signalPinState);
            if (DEBUG) {ESP_LOGI(TAG, "State: STOPPED");}
            return;

        case STATE_SYNC:
            next_alarm = T_SYNC_ON;
            state = STATE_PULSING;

            dataPinState = 0;
            gpio_set_level(DATA_LED, dataPinState);
            
            dataPulseNum = isStateAlternative ? NUM_PULSES + 1 : 0;

            signalPinState = 1;
            gpio_set_level(SIGNAL_LED, signalPinState);
            if (DEBUG) {ESP_LOGI(TAG, "State: SYNC");}
            break;

        case STATE_PULSING:
            if (signalPinState == 1) {
                signalPinState = 0;
                gpio_set_level(SIGNAL_LED, signalPinState);
            }
            
            if (dataPinState == 1) {
                next_alarm = PULSE_LOW_LEN;
                dataPinState = 0;
            } else {
                isStateAlternative ? dataPulseNum-- : dataPulseNum++;
                next_alarm = T_ON_n(dataPulseNum);
                dataPinState = 1;
            }
            
            bool reached_end = isStateAlternative ? dataPulseNum <= 0
                                                  : dataPulseNum >= NUM_PULSES + 1;
            if (!reached_end) {
                if (DEBUG) {ESP_LOGI(TAG, "State: PULSING, dataPulseNum: %d", dataPulseNum);}
                break;
            }
            state = STATE_PAUSE;
            // fallthrough

        case STATE_PAUSE:
            dataPinState = 0;
            dataPulseNum = isStateAlternative ? NUM_PULSES + 1 : 0;

            next_alarm = PULSE_PAUSE_LEN;
            state = STATE_SYNC;
            if (DEBUG) {ESP_LOGI(TAG, "State: PAUSE");}
            break;
    }

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = next_alarm,
    };
    if (next_alarm > 1200) {
        yield_wdg = true;
    }
    gptimer_set_raw_count(gptimer, 0);
    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_start(gptimer);
    gpio_set_level(DATA_LED, dataPinState);
}


static void IRAM_ATTR ISR_SELECT_Handler(void* arg)
{
    static volatile uint64_t lastTrigger = 0;
    uint64_t now = esp_timer_get_time() / 1000; // ms

    if (now - lastTrigger < DEBOUNCE_CD) return;
    lastTrigger = now;

    isStateAlternative = !isStateAlternative;

    if (isEnabled) {

        stopStateMachine();
        startStateMachine();
    }
}

static void IRAM_ATTR ISR_ENABLE_Handler(void* arg)
{
    static volatile uint64_t lastTrigger = 0;
    uint64_t now = esp_timer_get_time() / 1000; // ms

    if (now - lastTrigger < DEBOUNCE_CD) return;
    lastTrigger = now;

    if (isEnabled) {
        stopStateMachine();
    } else {
        startStateMachine();
    }
}

void configure_gpios(void)
{
    gpio_config_t input_conf = {
        .pin_bit_mask = (1ULL << RED_OUTPUT_ENABLE_PIN) | (1ULL << GREEN_OUTPUT_SELECT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&input_conf);

    gpio_config_t output_conf = {
        .pin_bit_mask = (1ULL << DATA_LED) | (1ULL << SIGNAL_LED) | (1ULL << EXTRA_GND),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&output_conf);

    gpio_set_level(EXTRA_GND, 0);

    for (int i = 0; i < 3; i++) {
        gpio_set_level(DATA_LED, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(DATA_LED, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    gpio_set_level(DATA_LED, 0);
    gpio_set_level(SIGNAL_LED, 0);
}

void configure_gptimer(void)
{
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };

    gptimer_new_timer(&timer_config, &gptimer);

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_cb,
    };

    gptimer_register_event_callbacks(gptimer, &cbs, NULL);
    gptimer_enable(gptimer);

    gptimer_alarm_config_t alarm_config = {
        .alarm_count = PULSE_PAUSE_LEN,
    };

    gptimer_set_alarm_action(gptimer, &alarm_config);
    gptimer_start(gptimer);
}

void configure_button_interrupts(void)
{
    // Install ISR service (1x per project)
    gpio_install_isr_service(0);

    // Configure pins as input with pull-up
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GREEN_OUTPUT_SELECT_PIN) | (1ULL << RED_OUTPUT_ENABLE_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE, // FALLING
    };
    gpio_config(&io_conf);

    // Attach ISRs
    gpio_isr_handler_add(GREEN_OUTPUT_SELECT_PIN, ISR_SELECT_Handler, NULL);
    gpio_isr_handler_add(RED_OUTPUT_ENABLE_PIN, ISR_ENABLE_Handler, NULL);
}

void checkStateChanges(void)
{
    static bool lastAlternative = false;
    static bool lastEnabled = false;

    // Check for change in isStateAlternative
    if (isStateAlternative != lastAlternative) {
        ESP_LOGI(TAG, "isStateAlternative changed: %s",
                 isStateAlternative ? "true" : "false");
        lastAlternative = isStateAlternative;
    }

    // Check for change in isEnabled
    if (isEnabled != lastEnabled) {
        ESP_LOGI(TAG, "isEnabled changed: %s",
                 isEnabled ? "true" : "false");
        lastEnabled = isEnabled;
    }
}

void stateMachineTask(void *pvParameter) {
    while (1) {
        // Block forever until timer ISR notifies this task
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Run your state machine
        stateMachine();
        if (DEBUG) {checkStateChanges();}
    }
}

void app_main(void)
{
    configure_gpios();

    // while (1) {
    //     if (timerTriggered) {
    //         timerTriggered = false;
    //         stateMachine();
    //     }
    //     if (DEBUG) {checkStateChanges();}
    //     if (yield_wdg) {
    //         yield_wdg = false;
    //         ESP_LOGI(TAG, "Yielding to watchdog...");
    //         vTaskDelay(1);
    //     }
    // }
    xTaskCreate(
        stateMachineTask,
        "stateMachineTask",
        4096,
        NULL,
        5,
        &stateTaskHandle
    );
    taskYIELD();
    configure_gptimer();
    configure_button_interrupts();
}

void startStateMachine(void)
{
    isEnabled = true;
    state = STATE_SYNC;

    BaseType_t higherPriorityWoken = pdFALSE;
    xTaskNotifyFromISR(stateTaskHandle, 0, eNoAction, &higherPriorityWoken);
    portYIELD_FROM_ISR(higherPriorityWoken);
}

void stopStateMachine(void)
{
    isEnabled = false;
    state = STATE_STOPPED;

    BaseType_t higherPriorityWoken = pdFALSE;
    xTaskNotifyFromISR(stateTaskHandle, 0, eNoAction, &higherPriorityWoken);
    portYIELD_FROM_ISR(higherPriorityWoken);
}