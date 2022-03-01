/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "lvgl.h"

/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. This example uses GPIO18/19/4/5 as LEDC output,
 *    and it will change the duty repeatedly.
 *
 * 5. GPIO18/19 are from high speed channel group.
 *    GPIO4/5 are from low speed channel group.
 *
 */

#define NUM_OF_SPIN_TASKS 6
#define SPIN_ITER 500000 // Actual CPU cycles used will depend on compiler optimization
#define SPIN_TASK_PRIO 2
#define ADC_TASK_PRIO 2
#define ADC_TICKS pdMS_TO_TICKS(1000)
#define LEDC_TASK_PRIO 3
#define LEDC_TICKS pdMS_TO_TICKS(1000)
#define ARRAY_SIZE_OFFSET 5 // Increase this if print_real_time_stats returns ESP_ERR_INVALID_SIZE

static char task_names[NUM_OF_SPIN_TASKS][configMAX_TASK_NAME_LEN];
static SemaphoreHandle_t sync_adc_task;
static SemaphoreHandle_t sync_ledc_task;

#if CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER LEDC_TIMER_0
#define LEDC_HS_MODE LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO (18)
#define LEDC_HS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO (19)
#define LEDC_HS_CH1_CHANNEL LEDC_CHANNEL_1
#endif
#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
#define LEDC_LS_CH0_GPIO (18)
#define LEDC_LS_CH0_CHANNEL LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO (19)
#define LEDC_LS_CH1_CHANNEL LEDC_CHANNEL_1
#endif
#define LEDC_LS_CH2_GPIO (3)
#define LEDC_LS_CH2_CHANNEL LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO (4)
#define LEDC_LS_CH3_CHANNEL LEDC_CHANNEL_3
#define LEDC_LS_CH4_GPIO (5)
#define LEDC_LS_CH4_CHANNEL LEDC_CHANNEL_4

#define LEDC_TEST_CH_NUM (4)
#define LEDC_TEST_DUTY (4000)
#define LEDC_TESTA_DUTY (0)
#define LEDC_TEST_FADE_TIME (50)

#define NUMS 8
#define TIMES 512
#define GET_UNIT(x) ((x >> 3) & 0x1)

#if CONFIG_IDF_TARGET_ESP32
#define ADC_RESULT_BYTE 2
#define ADC_CONV_LIMIT_EN 1                  // For ESP32, this should always be set to 1
#define ADC_CONV_MODE ADC_CONV_SINGLE_UNIT_1 // ESP32 only supports ADC1 DMA mode
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE1
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_RESULT_BYTE 2
#define ADC_CONV_LIMIT_EN 0
#define ADC_CONV_MODE ADC_CONV_BOTH_UNIT
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#elif CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
#define ADC_RESULT_BYTE 4
#define ADC_CONV_LIMIT_EN 0
#define ADC_CONV_MODE ADC_CONV_ALTER_UNIT // ESP32C3 only supports alter mode
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_RESULT_BYTE 4
#define ADC_CONV_LIMIT_EN 0
#define ADC_CONV_MODE ADC_CONV_BOTH_UNIT
#define ADC_OUTPUT_TYPE ADC_DIGI_OUTPUT_FORMAT_TYPE2
#endif

#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32H2
static uint16_t adc1_chan_mask = BIT(2) | BIT(3);
static uint16_t adc2_chan_mask = BIT(0);
static adc_channel_t channel[3] = {ADC1_CHANNEL_2, ADC1_CHANNEL_3, (ADC2_CHANNEL_0 | 1 << 3)};
#endif
#if CONFIG_IDF_TARGET_ESP32S2
static uint16_t adc1_chan_mask = BIT(2) | BIT(3);
static uint16_t adc2_chan_mask = BIT(0);
static adc_channel_t channel[3] = {ADC1_CHANNEL_2, ADC1_CHANNEL_3, (ADC2_CHANNEL_0 | 1 << 3)};
#endif
#if CONFIG_IDF_TARGET_ESP32
static uint16_t adc1_chan_mask = BIT(7);
static uint16_t adc2_chan_mask = 0;
static adc_channel_t channel[1] = {ADC1_CHANNEL_7};
#endif

static const char *TAG = "ADC DMA";
/*
static void adc_task(void *arg)
{
    esp_err_t ret;
    int adc1_reading[3] = {0xcc};
    // int adc2_reading[1] = {0xcc};

    const char TAG_CH[][10] = {"ADC1_CH2", "ADC1_CH3","ADC1_CH4", "ADC2_CH0"};

    adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    // adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
    // adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_0);
    // adc2_config_channel_atten(ADC2_CHANNEL_0, ADC_ATTEN_DB_0);

    // int n = 20;
    while (1) {

        adc1_reading[0] = adc1_get_raw(ADC1_CHANNEL_0);
        // adc1_reading[1] = adc1_get_raw(ADC1_CHANNEL_3);
        // adc1_reading[2] = adc1_get_raw(ADC1_CHANNEL_4);

        for (int i = 0; i < 1; i++) {
            ESP_LOGI(TAG_CH[i], "%d", adc1_reading[i]);
        }

        vTaskDelay(pdMS_TO_TICKS(50));

        // ret = adc2_get_raw(ADC2_CHANNEL_0, ADC_WIDTH_BIT_12, &adc2_reading[0]);
        // assert(ret == ESP_OK);
        // ESP_LOGI(TAG_CH[3], "%x", adc2_reading[0]);
    }
}
*/

static void continuous_adc_init(uint16_t adc1_chan_mask, uint16_t adc2_chan_mask, adc_channel_t *channel, uint8_t channel_num)
{
    esp_err_t ret = ESP_OK;
    assert(ret == ESP_OK);

    adc_digi_init_config_t adc_dma_config = {
        .max_store_buf_size = 1024,
        .conv_num_each_intr = 256,
        .adc1_chan_mask = adc1_chan_mask,
        .adc2_chan_mask = adc2_chan_mask,
    };
    ret = adc_digi_initialize(&adc_dma_config);
    assert(ret == ESP_OK);

    adc_digi_pattern_table_t adc_pattern[10] = {0};

    // Do not set the sampling frequency out of the range between `SOC_ADC_SAMPLE_FREQ_THRES_LOW` and `SOC_ADC_SAMPLE_FREQ_THRES_HIGH`
    adc_digi_config_t dig_cfg = {
        .conv_limit_en = 0,
        .conv_limit_num = 250,
        .sample_freq_hz = 615,
    };

    dig_cfg.adc_pattern_len = channel_num;
    for (int i = 0; i < channel_num; i++)
    {
        uint8_t unit = ((channel[i] >> 3) & 0x1);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
    }
    dig_cfg.adc_pattern = adc_pattern;
    ret = adc_digi_controller_config(&dig_cfg);
    assert(ret == ESP_OK);
}

static bool check_valid_data(const adc_digi_output_data_t *data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2)
        return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit))
        return false;

    return true;
}

static void adc_task(void *arg)
{
    esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[TIMES] = {0};
    memset(result, 0xcc, TIMES);

    uint16_t adc1_chan_mask = BIT(0) | BIT(1);
    uint16_t adc2_chan_mask = BIT(0);
    // adc_channel_t channel[3] = {ADC1_CHANNEL_0, ADC1_CHANNEL_1, (ADC2_CHANNEL_0 | 1 << 3)};
    adc_channel_t channel[1] = {ADC1_CHANNEL_0};

    continuous_adc_init(adc1_chan_mask, adc2_chan_mask, channel, sizeof(channel) / sizeof(adc_channel_t));
    ret = adc_digi_filter_enable(2, true);
    adc_digi_start();

    int n = 20;
    // while (1)
    // {
    //     ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
    //     for (int i = 0; i < ret_num; i += 4)
    //     {
    //         adc_digi_output_data_t *p = (void *)&result[i];
    //         if (check_valid_data(p))
    //         {
    //             printf("%d:ADC%d_CH%d: %x\n", ret_num, p->type2.unit + 1, p->type2.channel, p->type2.data);
    //         }
    //         else
    //         {
    //             printf("Invalid data [%d_%d_%x]\n", p->type2.unit + 1, p->type2.channel, p->type2.data);
    //         }
    //     }
    //     vTaskDelay(pdMS_TO_TICKS(500));
    //     // If you see task WDT in this task, it means the conversion is too fast for the task to handle
    // }
    while (1)
    {
        int max1 = 0, max2 = 0, min1 = 4096, min2 = 4096, tempnum = 0;
        float s_out = 0;
        int s_addp = 0;

        for (int sin = 0; sin < NUMS; sin++)
        {
            int s_add = 0;
            int s_num = 0;
            int s_temp = 0;
            int s_outs = 0;
            int16_t N = ret_num/4;
            int16_t value_buf[N];

            ret = adc_digi_read_bytes(result, TIMES, &ret_num, ADC_MAX_DELAY);
            if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE)
            {
                if (ret == ESP_ERR_INVALID_STATE)
                {
                    /**
                     * @note 1
                     * Issue:
                     * As an example, we simply print the result out, which is super slow. Therefore the conversion is too
                     * fast for the task to handle. In this condition, some conversion results lost.
                     *
                     * Reason:
                     * When this error occurs, you will usually see the task watchdog timeout issue also.
                     * Because the conversion is too fast, whereas the task calling `adc_digi_read_bytes` is slow.
                     * So `adc_digi_read_bytes` will hardly block. Therefore Idle Task hardly has chance to run. In this
                     * example, we add a `vTaskDelay(1)` below, to prevent the task watchdog timeout.
                     *
                     * Solution:
                     * Either decrease the conversion speed, or increase the frequency you call `adc_digi_read_bytes`
                     */
                }

                // ESP_LOGI("TASK:", "ret is %x, ret_num is %d", ret, ret_num);
                for (int i = 0; i < ret_num; i += ADC_RESULT_BYTE)
                {
                    adc_digi_output_data_t *p = (void *)&result[i];
                    if (ADC_CONV_MODE == ADC_CONV_BOTH_UNIT || ADC_CONV_MODE == ADC_CONV_ALTER_UNIT)
                    {
                        if (check_valid_data(p))
                        {
                            // ESP_LOGI(TAG, "Unit: %d,_Channel: %d, Value: %d", p->type2.unit + 1, p->type2.channel, p->type2.data);
                            s_temp = p->type2.data;
                            value_buf[s_num]=s_temp;
                            s_add += s_temp;
                            s_num++;
                            //ESP_LOGI(TAG, "TEMP: %d", s_temp);
                        }
                        else
                        {
                            // abort();
                            ESP_LOGI(TAG, "Invalid data [%d_%d_%x]", p->type2.unit + 1, p->type2.channel, p->type2.data);
                        }
                    }
                }
//***********************************************************
                int16_t i, j, temp;
                // for (count = 0; count < N; count++)
                // {
                //     value_buf[count] = get_ad();
                //     delay();
                // }
                for (j = 0; j < N - 1; j++)
                {
                    for (i = 0; i < N - j; i++)
                    {
                        if (value_buf[i] > value_buf[i + 1])
                        {
                            temp = value_buf[i];
                            value_buf[i] = value_buf[i + 1];
                            value_buf[i + 1] = temp;
                        }
                    }
                }
                // for (int k = 0; k < N; k++)
                // {
                //     printf("%d ", value_buf[k]);
                // }
                // printf("\r\n");
                s_outs = value_buf[(N) / 2];
//**************************************************************/
                //s_outs = s_add / s_num;
                ESP_LOGI(TAG, "OUT: %d,NUM: %d", s_outs,s_num);
                s_addp+=s_outs;
                // if (max2 <= s_outs)
                // {
                //     max2 = s_outs;
                //     if (max1 <= max2)
                //     {
                //         tempnum = max1;
                //         max1 = max2;
                //         max2 = tempnum;
                //     }
                // }
                // if (min2 >= s_outs)
                // {
                //     min2 = s_outs;
                //     if (min1 >= min2)
                //     {
                //         tempnum = min1;
                //         min1 = min2;
                //         min2 = tempnum;
                //     }
                // }
            // See `note 1`
            vTaskDelay(2);
            }
            else if (ret == ESP_ERR_TIMEOUT)
            {
                /**
                 * ``ESP_ERR_TIMEOUT``: If ADC conversion is not finished until Timeout, you'll get this return error.
                 * Here we set Timeout ``portMAX_DELAY``, so you'll never reach this branch.
                 */
                ESP_LOGW(TAG, "No data, increase timeout or reduce conv_num_each_intr");
                vTaskDelay(1000);
            }
        }
            // s_out = (float)(s_addp-max1-max2-min1-min2) / (float)(NUMS-4);
            s_out = (float)s_addp / (float)NUMS;
            float v_out=(float)s_out/4096*3;
            // printf("OUT: %f,RV: %f,V: %f", s_out,v_out,v_out*2);
            // ESP_LOGI(TAG, "OUT: %d,RV: %f,V: %f ,L:%f", s_out,v_out,v_adc,((float)s_out-(float)(s_out-125)*88/3088)/100);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    adc_digi_stop();
    ret = adc_digi_deinitialize();
    assert(ret == ESP_OK);
}

void ledc_task(void)
{
    int ch;

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,           // timer index
        .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
#ifdef CONFIG_IDF_TARGET_ESP32
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);
#endif
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
#if CONFIG_IDF_TARGET_ESP32
        {.channel = LEDC_HS_CH0_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_HS_CH0_GPIO,
         .speed_mode = LEDC_HS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_HS_TIMER},
        {.channel = LEDC_HS_CH1_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_HS_CH1_GPIO,
         .speed_mode = LEDC_HS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_HS_TIMER},
#elif CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3
        {.channel = LEDC_LS_CH0_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_LS_CH0_GPIO,
         .speed_mode = LEDC_LS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_LS_TIMER},
    // {
    //     .channel    = LEDC_LS_CH1_CHANNEL,
    //     .duty       = 0,
    //     .gpio_num   = LEDC_LS_CH1_GPIO,
    //     .speed_mode = LEDC_LS_MODE,
    //     .hpoint     = 0,
    //     .timer_sel  = LEDC_LS_TIMER
    // },
#endif
        {.channel = LEDC_LS_CH2_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_LS_CH2_GPIO,
         .speed_mode = LEDC_LS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_LS_TIMER},
        {.channel = LEDC_LS_CH3_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_LS_CH3_GPIO,
         .speed_mode = LEDC_LS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_LS_TIMER},
        {.channel = LEDC_LS_CH4_CHANNEL,
         .duty = 0,
         .gpio_num = LEDC_LS_CH4_GPIO,
         .speed_mode = LEDC_LS_MODE,
         .hpoint = 0,
         .timer_sel = LEDC_LS_TIMER},
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++)
    {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);

    while (1)
    {
        // printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
        //     ledc_fade_start(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        // }
        // vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        // printf("2. LEDC fade down to duty = %d\n", LEDC_TESTA_DUTY);
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_TESTA_DUTY, LEDC_TEST_FADE_TIME);
        //     ledc_fade_start(ledc_channel[ch].speed_mode,
        //             ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
        // }
        // vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

        for (ch = 1; ch < LEDC_TEST_CH_NUM; ch++)
        {
            // printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                                    ledc_channel[ch].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                            ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
            vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
            // printf("2. LEDC fade down to duty = %d\n", LEDC_TESTA_DUTY);
            ledc_set_fade_with_time(ledc_channel[ch].speed_mode,
                                    ledc_channel[ch].channel, LEDC_TESTA_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[ch].speed_mode,
                            ledc_channel[ch].channel, LEDC_FADE_NO_WAIT);
            vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
            // printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
            ledc_set_fade_with_time(ledc_channel[0].speed_mode,
                                    ledc_channel[0].channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[0].speed_mode,
                            ledc_channel[0].channel, LEDC_FADE_NO_WAIT);
            vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
            // printf("2. LEDC fade down to duty = %d\n", LEDC_TESTA_DUTY);
            ledc_set_fade_with_time(ledc_channel[0].speed_mode,
                                    ledc_channel[0].channel, LEDC_TESTA_DUTY, LEDC_TEST_FADE_TIME);
            ledc_fade_start(ledc_channel[0].speed_mode,
                            ledc_channel[0].channel, LEDC_FADE_NO_WAIT);
            vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
        }

        // printf("3. LEDC set duty = %d without fade\n", LEDC_TEST_DUTY);
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, LEDC_TEST_DUTY);
        //     ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        // }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);

        // printf("4. LEDC set duty = 0 without fade\n");
        // for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        //     ledc_set_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, 0);
        //     ledc_update_duty(ledc_channel[ch].speed_mode, ledc_channel[ch].channel);
        // }
        // vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void app_main(void)
{
    // Allow other core to finish initialization
    vTaskDelay(pdMS_TO_TICKS(100));

    // Create semaphores to synchronize
    //  sync_spin_task = xSemaphoreCreateCounting(NUM_OF_SPIN_TASKS, 0);
    sync_adc_task = xSemaphoreCreateBinary();
    sync_ledc_task = xSemaphoreCreateBinary();

    // Create spin tasks
    //  for (int i = 0; i < NUM_OF_SPIN_TASKS; i++) {
    //      snprintf(task_names[i], configMAX_TASK_NAME_LEN, "spin%d", i);
    //      xTaskCreatePinnedToCore(spin_task, task_names[i], 1024, NULL, SPIN_TASK_PRIO, NULL, tskNO_AFFINITY);
    //  }

    // Create and adc stats task
    xTaskCreatePinnedToCore(adc_task, "stats", 4096, NULL, ADC_TASK_PRIO, NULL, tskNO_AFFINITY);
    xSemaphoreGive(sync_adc_task);

    // Create and ledc stats task
    xTaskCreatePinnedToCore(ledc_task, "stats", 4096, NULL, LEDC_TASK_PRIO, NULL, tskNO_AFFINITY);
    xSemaphoreGive(sync_ledc_task);
}
