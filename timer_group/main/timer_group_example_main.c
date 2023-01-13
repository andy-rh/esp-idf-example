#if 0
/* General Purpose Timer example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds

typedef struct {
    int timer_group;
    int timer_idx;
    int alarm_interval;
    bool auto_reload;
} example_timer_info_t;

/**
 * @brief A sample structure to pass events from the timer ISR to task
 *
 */
typedef struct {
    example_timer_info_t info;
    uint64_t timer_counter_value;
} example_timer_event_t;

static xQueueHandle s_timer_queue;

/*
 * A simple helper function to print the raw timer counter value
 * and the counter value converted to seconds
 */
static void inline print_timer_counter(uint64_t counter_value)
{
    printf("Counter: 0x%08x%08x\r\n", (uint32_t) (counter_value >> 32),
           (uint32_t) (counter_value));
    printf("Time   : %.8f s\r\n", (double) counter_value / TIMER_SCALE);
}

static bool IRAM_ATTR timer_group_isr_callback(void *args)
{
    BaseType_t high_task_awoken = pdFALSE;
    example_timer_info_t *info = (example_timer_info_t *) args;

    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(info->timer_group, info->timer_idx);

    /* Prepare basic event data that will be then sent back to task */
    example_timer_event_t evt = {
        .info.timer_group = info->timer_group,
        .info.timer_idx = info->timer_idx,
        .info.auto_reload = info->auto_reload,
        .info.alarm_interval = info->alarm_interval,
        .timer_counter_value = timer_counter_value
    };

    if (!info->auto_reload) {
        timer_counter_value += info->alarm_interval * TIMER_SCALE;
        timer_group_set_alarm_value_in_isr(info->timer_group, info->timer_idx, timer_counter_value);
    }

    /* Now just send the event data back to the main program task */
    xQueueSendFromISR(s_timer_queue, &evt, &high_task_awoken);

    return high_task_awoken == pdTRUE; // return whether we need to yield at the end of ISR
}

/**
 * @brief Initialize selected timer of timer group
 *
 * @param group Timer Group number, index from 0
 * @param timer timer ID, index from 0
 * @param auto_reload whether auto-reload on alarm event
 * @param timer_interval_sec interval of alarm
 */
static void example_tg_timer_init(int group, int timer, bool auto_reload, int timer_interval_sec)
{
    //1、定时器初始化
    /* Select and initialize basic parameters of the timer */
    timer_config_t config = {
        .divider = TIMER_DIVIDER,//分频器.设置定时器中计数器计数的速度，设置范围在2-65536，这里设置为16，默认的时钟源APB频率为80MHZ，所以定时器的频率为80/16=5MHZ即0.2US
        .counter_dir = TIMER_COUNT_UP,//模式.递增
        .counter_en = TIMER_PAUSE,//暂停计数器计数
        .alarm_en = TIMER_ALARM_EN,//使能定时器警报
        .auto_reload = auto_reload,//设置计数器是否应该在定时器警报上使用 auto_reload 自动重载首个计数值，还是继续递增或递减******************是否自动重载
    }; // default clock source is APB
    timer_init(group, timer, &config);//初始化和配置定时器，定时器组0，1中的定时器0,1一共四个定时器

    //2、定时器控制
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(group, timer, 0);//指定定时器的首个计数值,写入定时器计数器的值为0
    
    //3、警报
    /* Configure the alarm value and the interrupt on alarm. */
    timer_set_alarm_value(group, timer, timer_interval_sec * TIMER_SCALE);//设置64位警报值**************************************计数器值的计算
    timer_enable_intr(group, timer);//使能定时器中断

    //4、处理中断事务
    example_timer_info_t *timer_info = calloc(1, sizeof(example_timer_info_t));
    timer_info->timer_group = group;
    timer_info->timer_idx = timer;
    timer_info->auto_reload = auto_reload;
    timer_info->alarm_interval = timer_interval_sec;
    timer_isr_callback_add(group, timer, timer_group_isr_callback, timer_info, 0);

    timer_start(group, timer);//使能(启动)定时器
}

void app_main(void)
{
    s_timer_queue = xQueueCreate(10, sizeof(example_timer_event_t));

    example_tg_timer_init(TIMER_GROUP_0, TIMER_0, 1, 3);
    example_tg_timer_init(TIMER_GROUP_1, TIMER_1, 0, 5);//false

    while (1) {
        example_timer_event_t evt;
        xQueueReceive(s_timer_queue, &evt, portMAX_DELAY);

        /* Print information that the timer reported an event */
        if (evt.info.auto_reload) {
            printf("Timer Group with auto reload\n");
        } else {
            printf("Timer Group without auto reload\n");
        }
        printf("Group[%d], timer[%d] alarm event\n", evt.info.timer_group, evt.info.timer_idx);

        /* Print the timer values passed by event */
        printf("------- EVENT TIME --------\n");
        print_timer_counter(evt.timer_counter_value);

        /* Print the timer values as visible by this task */
        printf("-------- TASK TIME --------\n");
        uint64_t task_counter_value;
        timer_get_counter_value(evt.info.timer_group, evt.info.timer_idx, &task_counter_value);
        print_timer_counter(task_counter_value);
    }
}
#endif

#if 1
/********************************************硬件定时器*****************************************/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/timer.h"
#include "driver/gpio.h"

#define TIMER_DIVIDER         (16)  //  Hardware timer clock divider
// convert counter value to seconds 80*1 000 000 HZ / 16 = 5 * 1 000 000 HZ
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  

#define GPIO_NUM_26 26//IO口26
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<GPIO_NUM_26)  // 配置GPIO_OUT位寄存器

__uint8_t led_flag = 0;

void gpio_init(void)
{
    gpio_config_t io_conf;  // 定义一个gpio_config类型的结构体，下面的都算对其进行的配置

    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;  // 禁止中断  
    io_conf.mode = GPIO_MODE_OUTPUT;            // 选择输出模式
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // 配置GPIO_OUT寄存器
    io_conf.pull_down_en = 0;                   // 禁止下拉
    io_conf.pull_up_en = 0;                     // 禁止上拉

    gpio_config(&io_conf);                      // 最后配置使能
}

void IRAM_ATTR timer_group1_isr(void *para){
	    //获取定时器分组1中的哪一个定时器产生了中断
	    uint32_t timer_intr = timer_group_get_intr_status_in_isr(TIMER_GROUP_1); //仅在ISR中获取中断状态
	    if (timer_intr & TIMER_INTR_T0) {//定时器1分组的0号定时器产生中断
	        /*清除中断状态*/
	        timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_0);
	        /*重新使能定时器中断*/
	        timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_0);//警报一旦触发后，警报将自动关闭，需要重新使能以再次触发
	    }
        /*led交替闪烁，时间为定时器时间1s*/
        if(led_flag==0){
        	led_flag =1;
        	gpio_set_level(GPIO_NUM_26, 0);//打开LED
        }else{
        	led_flag=0;
        	gpio_set_level(GPIO_NUM_26, 1);//关闭LED
        }
}

static void timer_init_test()
{
    //1、定时器初始化
    timer_config_t config = {
//分频器.设置定时器中计数器计数的速度，设置范围在2-65536，这里设置为16，默认的时钟源APB频率为80MHZ，所以定时器的频率为80/16=5MHZ即0.2US来一次脉冲
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,//模式.向上递增
        .counter_en = TIMER_PAUSE,//暂停计数器计数,调用timer_start时才开始计数
        .alarm_en = TIMER_ALARM_EN,//使能定时器警报,到达计数器设置的值进入中断
        .auto_reload = 1,//设置计数器是否应该在定时器警报上使用自动重载首个计数值，还是继续递增或递减
    }; // default clock source is APB
    timer_init(TIMER_GROUP_1, TIMER_0, &config);//初始化和配置定时器，定时器组0，1中的定时器0,1一共四个定时器

    //2、定时器控制
    timer_set_counter_value(TIMER_GROUP_1, TIMER_0, 0);//指定定时器的首个计数值,写入定时器计数器的值为0
    
    //3、警报
    /* Configure the alarm value and the interrupt on alarm. */
    //MS设置64位警报值,此函数的第三个变量为64位的变量，即此变量为计数器的值5 * 1 000 000HZ * 0.2us = 1 S
    timer_set_alarm_value(TIMER_GROUP_1, TIMER_0, 1 * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_1, TIMER_0);//使能定时器中断

    //4、处理中断事务
    timer_isr_register(TIMER_GROUP_1,TIMER_0,
			timer_group1_isr,  //定时器中断回调函数
			(void*)TIMER_0,    //传递给定时器回调函数的参数
			ESP_INTR_FLAG_IRAM, //把中断放到 IRAM 中
			NULL //调用成功以后返回中断函数的地址,一般用不到
    );

    timer_start(TIMER_GROUP_1, TIMER_0);//使能(启动)定时器
}

void app_main(void)
{
    gpio_init();//初始化GPIO

    timer_init_test();

    while (1) {

            vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

#endif