#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/header.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"

#ifdef ESP_PLATFORM
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// Pin Definitions for TB6612
#define M_LEFT_PWM   18
#define M_LEFT_IN1   5
#define M_LEFT_IN2   17

#define M_RIGHT_PWM  19
#define M_RIGHT_IN1  16
#define M_RIGHT_IN2  4

#define STBY_PIN     2  // Standby pin must be HIGH for the driver to work

#define TRACK_WIDTH 0.5 // meters
#define STRING_BUFFER_LEN 100
#define TIMEOUT_MS 500 // Stop motors if no msg for 0.5 seconds
#define PWM_FREQUENCY 400 // Hz
#define MIN_PWM 100 // Minimum PWM duty for the motors
#define MAX_PWM 150 // Maximum PWM duty for the motors

int64_t last_cmd_vel_time = 0;

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t debug_publisher;

geometry_msgs__msg__TwistStamped incoming_cmd_vel;
std_msgs__msg__String outcoming_debug;

char debug_buffer[STRING_BUFFER_LEN]; // Buffer for string messages

void init_hardware() 
{
    // Configure Direction and Standby Pins
    gpio_reset_pin(M_LEFT_IN1);  gpio_set_direction(M_LEFT_IN1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(M_LEFT_IN2);  gpio_set_direction(M_LEFT_IN2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(M_RIGHT_IN1); gpio_set_direction(M_RIGHT_IN1, GPIO_MODE_OUTPUT);
    gpio_reset_pin(M_RIGHT_IN2); gpio_set_direction(M_RIGHT_IN2, GPIO_MODE_OUTPUT);
    gpio_reset_pin(STBY_PIN);    gpio_set_direction(STBY_PIN, GPIO_MODE_OUTPUT);
    
    gpio_set_level(STBY_PIN, 1); // Enable driver

    // Configure PWM (LEDC)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    ledc_channel_config_t lcd_ch[2] = {
        {.gpio_num = M_LEFT_PWM,  .channel = LEDC_CHANNEL_0, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = M_RIGHT_PWM, .channel = LEDC_CHANNEL_1, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0}
    };
    for(int i=0; i<2; i++) ledc_channel_config(&lcd_ch[i]);
}

void init_publishers(rcl_node_t *node)
{	
	// Initialize the debug message
	outcoming_debug.data.data = debug_buffer;
	outcoming_debug.data.size = 0;
	outcoming_debug.data.capacity = STRING_BUFFER_LEN;

	rclc_publisher_init_default(&debug_publisher, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/debug");
}

void init_subscribers(rcl_node_t *node)
{
	RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_subscriber, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/cmd_vel"));
}

void set_motor_speed(int channel, int in1_pin, int in2_pin, float speed)
{
	float abs_speed = fabsf(speed);
	uint32_t duty = 0;

	if (abs_speed < 0.01) {
		duty = 0;
		gpio_set_level(in1_pin, 0); gpio_set_level(in2_pin, 0);
	} else {
		duty = MIN_PWM + (uint32_t)(abs_speed * (255 - MIN_PWM));
		if (duty > MAX_PWM) duty = MAX_PWM;

		if (speed > 0) {
            gpio_set_level(in1_pin, 1); gpio_set_level(in2_pin, 0);
        } else {
            gpio_set_level(in1_pin, 0); gpio_set_level(in2_pin, 1);
        }
	}

	ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
	ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

void cmd_vel_subscription_callback (const void * msgin)
{
	last_cmd_vel_time = esp_timer_get_time() / 1000; // Refresh the last command velocity time
	const geometry_msgs__msg__TwistStamped * msg = (const geometry_msgs__msg__TwistStamped *)msgin;

	float linear_x = msg->twist.linear.x;
	float angular_z = msg->twist.angular.z;

	sprintf(outcoming_debug.data.data, "CMD_VEL_RECEIVED %f %f", linear_x, angular_z);
	outcoming_debug.data.size = strlen(outcoming_debug.data.data);
	rcl_publish(&debug_publisher, (const void*)&outcoming_debug, NULL);

	// Kinematic model for differential drive robot
	float left_velocity = linear_x - (angular_z * TRACK_WIDTH / 2);
	float right_velocity = linear_x + (angular_z * TRACK_WIDTH / 2);

	set_motor_speed(LEDC_CHANNEL_0, M_LEFT_IN1, M_LEFT_IN2, left_velocity);
	set_motor_speed(LEDC_CHANNEL_1, M_RIGHT_IN1, M_RIGHT_IN2, right_velocity);

	sprintf(outcoming_debug.data.data, "LEFT_VELOCITY %f RIGHT_VELOCITY %f", left_velocity, right_velocity);
	outcoming_debug.data.size = strlen(outcoming_debug.data.data);
	rcl_publish(&debug_publisher, (const void*)&outcoming_debug, NULL);
}


void appMain(void *argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	printf("Rover app started\n");

	// Initialize hardware
	printf("Initializing hardware...\n");
	init_hardware();
	vTaskDelay(pdMS_TO_TICKS(500));

	// Wait for micro-ROS agent to be ready
	while (1) {
        // Ping the agent with a 100ms timeout, 1 attempt
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            printf("Agent detected! Connecting...\n");
            break; 
        }
        
        // If not found, wait 1 second and try again
        printf("Agent not found. Retrying in 1s...\n");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

	// create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "rover_node", "", &support));

	// Initialize publishers and subscribers
	init_publishers(&node);
	init_subscribers(&node);

	// Create executor
	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &incoming_cmd_vel,
		&cmd_vel_subscription_callback, ON_NEW_DATA));

	// Start the executor
	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

		// --- SAFETY STOP LOGIC ---
		int64_t now = esp_timer_get_time() / 1000;
		if(now - last_cmd_vel_time > TIMEOUT_MS) {
			set_motor_speed(LEDC_CHANNEL_0, M_LEFT_IN1, M_LEFT_IN2, 0);
			set_motor_speed(LEDC_CHANNEL_1, M_RIGHT_IN1, M_RIGHT_IN2, 0);

			sprintf(outcoming_debug.data.data, "SAFETY STOP - NO CMD_VEL RECEIVED FOR %dms", TIMEOUT_MS);
			outcoming_debug.data.size = strlen(outcoming_debug.data.data);
			rcl_publish(&debug_publisher, (const void*)&outcoming_debug, NULL);
		}
		
		usleep(10000);
	}

	// Free resources
	RCCHECK(rcl_publisher_fini(&debug_publisher, &node));
	RCCHECK(rcl_subscription_fini(&cmd_vel_subscriber, &node));
	RCCHECK(rcl_node_fini(&node));
}
