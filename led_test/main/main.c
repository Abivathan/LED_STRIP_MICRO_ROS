#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "esp_ws28xx.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <custom_msg/msg/led_data.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

#define LED_GPIO 42
#define LED_NUM 144

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
custom_msg__msg__LedData msg_pub;
custom_msg__msg__LedData msg_sub;

CRGB* ws2812_buffer;

double brightness = 0.0;
double fadeAmount = 0.01;
// volatile int32_t mode = 0;
volatile int32_t id_modes[][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0}}; 
// volatile int32_t r = 0;
// volatile int32_t g = 0;
// volatile int32_t b = 0;

typedef unsigned char byte;

// Define sections for ID 1 and ID 2
int32_t sections[][2] = {
    {0, 45}, {100, 144}, // ID 1 controls these sections
    {46, 100}            // ID 2 controls this section
};
// int32_t sec_mode[] = {{0,0},{0,0}};  // 0:Breathing, 1:Rainbow

uint16_t j[] = {0,0,0,0};

void rainbowCycle(int s);
byte *Wheel(byte WheelPosition);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&publisher, &msg_pub, NULL));
		msg_pub.id++;
	}
}

void led_off(int s)
{
    for (uint16_t i = sections[s][0]; i < sections[s][1]; i++){
        ws2812_buffer[i] = (CRGB){.r = 0, .g = 0, .b = 0};
    }
}

void static_color(int s)
{
    int32_t r = id_modes[s][1];
    int32_t g = id_modes[s][2];
    int32_t b = id_modes[s][3];

    for(uint16_t i = sections[s][0]; i < sections[s][1]; i++){
        ws2812_buffer[i] = (CRGB){.r = r, .g = g, .b = b};
    }
}

void breathing_effect(int s)
{   
    int32_t r = id_modes[s][1];
    int32_t g = id_modes[s][2];
    int32_t b = id_modes[s][3];

    if (brightness <= 0.0 || brightness >= 1.0) {
        fadeAmount = -fadeAmount; // Reverse the fade direction
    }
    brightness += fadeAmount;

    if (brightness > 1.0) brightness = 1.0;
    if (brightness < 0.0) brightness = 0.0;

    // int num_leds = sections[s][1] - sections[s][0];

    for(uint16_t i = sections[s][0]; i < sections[s][1]; i++){
        ws2812_buffer[i] = (CRGB){.r = r * brightness, .g = g * brightness, .b = b * brightness};
    }
}

void rainbowCycle(int s) 
{
    byte *c;
    uint16_t i;

    // for(j=0; j < 256; j++) {

    // if (mode != 3) {
    //     break;  
    // }

    if(j[s] >= 256){
        j[s] = 0;
    }

    int num_leds = sections[s][1] - sections[s][0];

    for(i= sections[s][0]; i < sections[s][1]; i++) {
        c = Wheel(((i * 256 / num_leds) + j[s]) & 255);
        ws2812_buffer[LED_NUM - 1 - i] = (CRGB){.r = *c, .g = *(c+1), .b = *(c+2)};
    }
    j[s]++;
    ws28xx_update();
    vTaskDelay(10 / portTICK_PERIOD_MS);
    // }
}

byte *Wheel(byte WheelPosition) {
    static byte c[3];

    if(WheelPosition < 85) {
        c[0] = WheelPosition * 3;
        c[1] = 255 - WheelPosition * 3;
        c[2] = 0;
    }
    else if(WheelPosition < 170) {
        WheelPosition -= 85;
        c[0] = 255 - WheelPosition * 3;
        c[1] = 0;
        c[2] = WheelPosition * 3;
    }
    else {
        WheelPosition -= 170;
        c[0] = 0;
        c[1] = WheelPosition * 3;
        c[2] = 255 - WheelPosition * 3;
    }

    return c;
}

void led_control_task(void * arg) {
    while (1) {

        for(int i = 0; i < 2; i++){ // Loop only for ID 1 and ID 2

            int mode = id_modes[i+1][0];
            // OFF
            if (mode == 0) {
                led_off(i);
                ws28xx_update();
            }
            // Static Colour
            else if (mode == 1) {
                static_color(i);
                ws28xx_update();
            }
            // Breathing Effect
            else if (mode == 2) {
                breathing_effect(i);
                ws28xx_update();
            }
            // Rainbow Flow
            else if (mode == 3) {
                rainbowCycle(i);
            }
        }
    }
}

void subscription_callback(const void * msgin)
{
	const custom_msg__msg__LedData * msg = (const custom_msg__msg__LedData *)msgin;
	// printf("Received: ID=%d, Mode=%d, RGB=(%d,%d,%d)\n",  msg->id, msg->mode, msg->r, msg->g, msg->b);

    if(msg->id == 1) {
        id_modes[1][0] = msg->mode;
        id_modes[1][1] = msg->r;
        id_modes[1][2] = msg->g;
        id_modes[1][3] = msg->b;
    }
    else if(msg->id == 2) {
        id_modes[2][0] = msg->mode;
        id_modes[2][1] = msg->r;
        id_modes[2][2] = msg->g;
        id_modes[2][3] = msg->b;
    }
}

void micro_ros_task(void * arg)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "esp32s3_pub_sub", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msg, msg, LedData),
		"led_data_publisher"));
		
	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(custom_msg, msg, LedData),
		"led_data_subscriber"));

	// create timer for publisher,
	rcl_timer_t timer;
	const unsigned int timer_timeout = 1000;
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

    // // create timer for Led Strip control,
	// rcl_timer_t timer_2;
	// const unsigned int timer_timeout_2 = 10;
	// RCCHECK(rclc_timer_init_default(
	// 	&timer_2,
	// 	&support,
	// 	RCL_MS_TO_NS(timer_timeout_2),
	// 	led_control_timer_callback));

	// create executor
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));
	
	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
    // RCCHECK(rclc_executor_add_timer(&executor, &timer_2));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_sub, &subscription_callback, ON_NEW_DATA));

	msg_pub.id = 0;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(100000);
	}

	// free resources
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

static size_t uart_port = UART_NUM_0;

void app_main(void) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(ws28xx_init(LED_GPIO, WS2812B, LED_NUM, &ws2812_buffer));

    // blink_led();
    // vTaskDelay(1000 / portTICK_PERIOD_MS);

    #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
        rmw_uros_set_custom_transport(
            true,
            (void *) &uart_port,
            esp32_serial_open,
            esp32_serial_close,
            esp32_serial_write,
            esp32_serial_read
        );
    #else
    #error micro-ROS transports misconfigured
    #endif  // RMW_UXRCE_TRANSPORT_CUSTOM

        xTaskCreate(micro_ros_task,
                "uros_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
        
        xTaskCreate(led_control_task,
                "led_control_task",
                CONFIG_MICRO_ROS_APP_STACK,
                NULL,
                CONFIG_MICRO_ROS_APP_TASK_PRIO,
                NULL);
}





//ros2  topic pub --once /led_data_subscriber custom_msg/msg/LedData "{mode: 2, id : 2 , r : 10, g : 20, b : 30}"
