// Imports
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

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


// LEFT FRONT
#define M_LF_PWM 25
#define M_LF_DIR 33
#define M_LF_SLP 32

// LEFT REAR
#define M_LR_PWM 21
#define M_LR_DIR 22
#define M_LR_SLP 23

// RIGHT FRONT
#define M_RF_PWM 4
#define M_RF_DIR 26
#define M_RF_SLP 19

// RIGHT REAR
#define M_RR_PWM 18
#define M_RR_DIR 17
#define M_RR_SLP 16

// LEFT FRONT ENCODER
#define ENC_LF_A 36
#define ENC_LF_B 39

// LEFT REAR ENCODER
#define ENC_LR_A 34
#define ENC_LR_B 35

// RIGHT FRONT ENCODER
#define ENC_RF_A 13
#define ENC_RF_B 5

// RIGHT REAR ENCODER
#define ENC_RR_A 14
#define ENC_RR_B 27

#define TRACK_WIDTH 0.5 // meters
#define STRING_BUFFER_LEN 100
#define TIMEOUT_MS 2000 // Stop motors if no msg for 2 seconds
#define PWM_FREQUENCY 20000 // Hz
#define DEADZONE 0.05 // m/s
#define MAX_SPEED 1.0f // m/s

// Forward declarations
void publish_debug(const char * msg);

// Encoder counts
volatile int32_t enc_lf = 0;
volatile int32_t enc_lr = 0;
volatile int32_t enc_rf = 0;
volatile int32_t enc_rr = 0;

// Global variables
int64_t last_cmd_vel_time = 0;
bool debug_publisher_ready = false;

// Pre-init log buffer — captures messages before the debug publisher is ready
#define LOG_BUFFER_COUNT 16
static char log_buffer[LOG_BUFFER_COUNT][STRING_BUFFER_LEN];
static int log_buffer_head = 0;

void log_msg(const char *msg) {
    if (debug_publisher_ready) {
        publish_debug(msg);
    } else {
        if (log_buffer_head < LOG_BUFFER_COUNT) {
            snprintf(log_buffer[log_buffer_head++], STRING_BUFFER_LEN, "%s", msg);
        }
    }
}

void flush_log_buffer(void) {
    for (int i = 0; i < log_buffer_head; i++) {
        publish_debug(log_buffer[i]);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    log_buffer_head = 0;
}

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t debug_publisher;
rcl_publisher_t left_whl_publisher;
rcl_publisher_t right_whl_publisher;

geometry_msgs__msg__TwistStamped incoming_cmd_vel;
std_msgs__msg__String outcoming_debug;
std_msgs__msg__String left_whl_msg;
std_msgs__msg__String right_whl_msg;

char debug_buffer[STRING_BUFFER_LEN];
char left_whl_buffer[STRING_BUFFER_LEN];
char right_whl_buffer[STRING_BUFFER_LEN];

// Encoder ISR handlers — count ticks on A channel (ANYEDGE = 32 CPR per motor)
static void IRAM_ATTR enc_lf_isr(void *arg) { enc_lf++; }
static void IRAM_ATTR enc_lr_isr(void *arg) { enc_lr++; }
static void IRAM_ATTR enc_rf_isr(void *arg) { enc_rf++; }
static void IRAM_ATTR enc_rr_isr(void *arg) { enc_rr++; }

void init_encoders()
{
    gpio_config_t io_conf = {
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << ENC_LF_A) | (1ULL << ENC_LF_B) |
                        (1ULL << ENC_LR_A) | (1ULL << ENC_LR_B) |
                        (1ULL << ENC_RF_A) | (1ULL << ENC_RF_B) |
                        (1ULL << ENC_RR_A) | (1ULL << ENC_RR_B)
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);

    gpio_set_intr_type(ENC_LF_A, GPIO_INTR_ANYEDGE);  gpio_isr_handler_add(ENC_LF_A, enc_lf_isr, NULL);
    gpio_set_intr_type(ENC_LR_A, GPIO_INTR_ANYEDGE);  gpio_isr_handler_add(ENC_LR_A, enc_lr_isr, NULL);
    gpio_set_intr_type(ENC_RF_A, GPIO_INTR_ANYEDGE);  gpio_isr_handler_add(ENC_RF_A, enc_rf_isr, NULL);
    gpio_set_intr_type(ENC_RR_A, GPIO_INTR_ANYEDGE);  gpio_isr_handler_add(ENC_RR_A, enc_rr_isr, NULL);
}

// Hardware initialization
void init_hardware() 
{
	// Configure direction and power pins as gpio outputs
    uint64_t pin_mask = (1ULL << M_LF_DIR) | (1ULL << M_LF_SLP) |
                    (1ULL << M_LR_DIR) | (1ULL << M_LR_SLP) |
                    (1ULL << M_RF_DIR) | (1ULL << M_RF_SLP) |
                    (1ULL << M_RR_DIR) | (1ULL << M_RR_SLP);
	gpio_config_t io_conf = {.mode = GPIO_MODE_OUTPUT, .pin_bit_mask = pin_mask};
	gpio_config(&io_conf);

	// Enable the drivers
	gpio_set_level(M_LF_SLP, 1); gpio_set_level(M_LR_SLP, 1);
	gpio_set_level(M_RF_SLP, 1); gpio_set_level(M_RR_SLP, 1);

    // Configure PWM (LEDC)
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // Configure 4 PWM Channels
    ledc_channel_config_t lcd_ch[4] = {
        {.gpio_num = M_LF_PWM, .channel = LEDC_CHANNEL_0, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = M_LR_PWM, .channel = LEDC_CHANNEL_1, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = M_RF_PWM, .channel = LEDC_CHANNEL_2, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = M_RR_PWM, .channel = LEDC_CHANNEL_3, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0}
    };
    for(int i=0; i<4; i++) ledc_channel_config(&lcd_ch[i]);

    init_encoders();
}


// Helper function to publish debug messages
void publish_debug(const char * msg) {
    int len = snprintf(debug_buffer, STRING_BUFFER_LEN, "%s", msg);
    if (len >= 0 && len < STRING_BUFFER_LEN) {
        outcoming_debug.data.size = len;
    } else {
        outcoming_debug.data.size = STRING_BUFFER_LEN - 1;
    }
    rcl_publish(&debug_publisher, (const void*)&outcoming_debug, NULL);
}

// Helper function to set the motor speed
void set_motor_speed(int channel, int dir_pin, float speed)
{
	// Get magnitude of the speed
	float abs_speed = fabsf(speed);

	// If the speed is less than the deadzone, set the duty cycle to 0
	if (abs_speed < DEADZONE) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
        return;
    }

	// Set duty cycle
	uint32_t duty = (uint32_t)((abs_speed / MAX_SPEED) * 255.0f);
	if (duty > 255) duty = 255;

	// Set direction
	gpio_set_level(dir_pin, (speed >= 0) ? 1 : 0);

	// Update the PWM hardware channel
    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}

// Callback function to convert twist message to motor speeds
void cmd_vel_subscription_callback (const void * msgin)
{
	// Refresh the last command velocity time
	last_cmd_vel_time = esp_timer_get_time() / 1000;
	
	// Extract the linear and angular velocities from the incoming message
	const geometry_msgs__msg__TwistStamped * msg = (const geometry_msgs__msg__TwistStamped *)msgin;
	float linear_x = msg->twist.linear.x;
	float angular_z = msg->twist.angular.z;

	// Kinematic model for differential drive robot
	float left_velocity = linear_x - (angular_z * TRACK_WIDTH / 2.0);
	float right_velocity = linear_x + (angular_z * TRACK_WIDTH / 2.0);

	// Set motor speed for LEFT SIDE MOTORS
	set_motor_speed(LEDC_CHANNEL_0, M_LF_DIR, left_velocity);
	set_motor_speed(LEDC_CHANNEL_1, M_LR_DIR, left_velocity);

	// Set motor speed for RIGHT SIDE MOTORS
	set_motor_speed(LEDC_CHANNEL_2, M_RF_DIR, right_velocity);
	set_motor_speed(LEDC_CHANNEL_3, M_RR_DIR, right_velocity);

	// Publish the left and right motor velocities
	char temp_buf[STRING_BUFFER_LEN];
	snprintf(temp_buf, STRING_BUFFER_LEN, "L: %.2f, R: %.2f", left_velocity, right_velocity);
    publish_debug(temp_buf);
}


void appMain(void *argument)
{
	log_msg("Rover app started");
	log_msg("Initializing hardware...");
	init_hardware();
	vTaskDelay(pdMS_TO_TICKS(500));

	while (1) {  // reconnection loop — retries the entire micro-ROS init on any failure

		rcl_allocator_t allocator = rcl_get_default_allocator();
		rclc_support_t support;
		rcl_node_t node;
		rclc_executor_t executor;
		bool support_ok = false, node_ok = false;
		bool pub_debug_ok = false, pub_left_ok = false, pub_right_ok = false;
		bool sub_ok = false, executor_ok = false;

		// Wait for agent with a single ping + fixed delay.
		// Multiple pings each open a temporary session and leave the agent in a
		// confused state when rclc_support_init tries to establish the real one.
		log_buffer_head = 0;
		while (rmw_uros_ping_agent(500, 1) != RMW_RET_OK) {
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
		log_msg("Agent detected, waiting for ready...");
		vTaskDelay(pdMS_TO_TICKS(2000));

		// --- INIT ---
		if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
			log_msg("support_init failed"); goto cleanup; }
		support_ok = true;
		vTaskDelay(pdMS_TO_TICKS(500));

		if (rclc_node_init_default(&node, "rover_node", "", &support) != RCL_RET_OK) {
			log_msg("node_init failed"); goto cleanup; }
		node_ok = true;
		vTaskDelay(pdMS_TO_TICKS(200));

		outcoming_debug.data.data = debug_buffer;
		outcoming_debug.data.size = 0;
		outcoming_debug.data.capacity = STRING_BUFFER_LEN;
		if (rclc_publisher_init_default(&debug_publisher, &node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/debug") != RCL_RET_OK) {
			log_msg("debug_publisher init failed"); goto cleanup; }
		pub_debug_ok = true;
		debug_publisher_ready = true;
		flush_log_buffer();
		vTaskDelay(pdMS_TO_TICKS(200));

		left_whl_msg.data.data = left_whl_buffer;
		left_whl_msg.data.size = 0;
		left_whl_msg.data.capacity = STRING_BUFFER_LEN;
		if (rclc_publisher_init_default(&left_whl_publisher, &node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/left_whl") != RCL_RET_OK) {
			log_msg("left_whl_publisher init failed"); goto cleanup; }
		pub_left_ok = true;
		vTaskDelay(pdMS_TO_TICKS(200));

		right_whl_msg.data.data = right_whl_buffer;
		right_whl_msg.data.size = 0;
		right_whl_msg.data.capacity = STRING_BUFFER_LEN;
		if (rclc_publisher_init_default(&right_whl_publisher, &node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/right_whl") != RCL_RET_OK) {
			log_msg("right_whl_publisher init failed"); goto cleanup; }
		pub_right_ok = true;
		vTaskDelay(pdMS_TO_TICKS(200));

		if (rclc_subscription_init_best_effort(&cmd_vel_subscriber, &node,
				ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/cmd_vel") != RCL_RET_OK) {
			log_msg("cmd_vel_subscriber init failed"); goto cleanup; }
		sub_ok = true;
		vTaskDelay(pdMS_TO_TICKS(200));

		if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
			log_msg("executor_init failed"); goto cleanup; }
		executor_ok = true;
		if (rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &incoming_cmd_vel,
				&cmd_vel_subscription_callback, ON_NEW_DATA) != RCL_RET_OK) {
			log_msg("executor_add_subscription failed"); goto cleanup; }

		log_msg("micro-ROS fully initialized");

		// --- MAIN LOOP ---
		{
			bool in_safety_stop = false;
			last_cmd_vel_time = esp_timer_get_time() / 1000;
			int64_t last_enc_publish_time = esp_timer_get_time() / 1000;
			int consecutive_publish_failures = 0;

			while (1) {
				rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

				// --- ENCODER PUBLISH (20Hz) ---
				int64_t now_enc = esp_timer_get_time() / 1000;
				if (now_enc - last_enc_publish_time >= 50) {
					int llen = snprintf(left_whl_buffer,  STRING_BUFFER_LEN, "lf:%d,lr:%d", enc_lf, enc_lr);
					int rlen = snprintf(right_whl_buffer, STRING_BUFFER_LEN, "rf:%d,rr:%d", enc_rf, enc_rr);
					left_whl_msg.data.size  = (llen >= STRING_BUFFER_LEN) ? STRING_BUFFER_LEN - 1 : llen;
					right_whl_msg.data.size = (rlen >= STRING_BUFFER_LEN) ? STRING_BUFFER_LEN - 1 : rlen;
					rcl_ret_t pub_ret = rcl_publish(&left_whl_publisher, &left_whl_msg, NULL);
					rcl_publish(&right_whl_publisher, &right_whl_msg, NULL);
					last_enc_publish_time = now_enc;

					if (pub_ret != RCL_RET_OK) {
						consecutive_publish_failures++;
						if (consecutive_publish_failures >= 5) {
							log_msg("Agent lost, reconnecting...");
							goto cleanup;
						}
					} else {
						consecutive_publish_failures = 0;
					}
				}

				// --- SAFETY STOP LOGIC ---
				int64_t now = esp_timer_get_time() / 1000;
				if (now - last_cmd_vel_time > TIMEOUT_MS) {
					if (!in_safety_stop) {
						set_motor_speed(LEDC_CHANNEL_0, M_LF_DIR, 0);
						set_motor_speed(LEDC_CHANNEL_1, M_LR_DIR, 0);
						set_motor_speed(LEDC_CHANNEL_2, M_RF_DIR, 0);
						set_motor_speed(LEDC_CHANNEL_3, M_RR_DIR, 0);
						char temp_buf[STRING_BUFFER_LEN];
						snprintf(temp_buf, STRING_BUFFER_LEN, "SAFETY STOP - NO CMD_VEL FOR %dms", TIMEOUT_MS);
						publish_debug(temp_buf);
						in_safety_stop = true;
					}
				} else {
					in_safety_stop = false;
				}
				vTaskDelay(pdMS_TO_TICKS(10));
			}
		}

		cleanup:
		log_msg("Cleaning up, will reconnect...");
		debug_publisher_ready = false;
		set_motor_speed(LEDC_CHANNEL_0, M_LF_DIR, 0);
		set_motor_speed(LEDC_CHANNEL_1, M_LR_DIR, 0);
		set_motor_speed(LEDC_CHANNEL_2, M_RF_DIR, 0);
		set_motor_speed(LEDC_CHANNEL_3, M_RR_DIR, 0);
		if (executor_ok) rclc_executor_fini(&executor);
		if (sub_ok)      rcl_subscription_fini(&cmd_vel_subscriber, &node);
		if (pub_right_ok) rcl_publisher_fini(&right_whl_publisher, &node);
		if (pub_left_ok)  rcl_publisher_fini(&left_whl_publisher, &node);
		if (pub_debug_ok) rcl_publisher_fini(&debug_publisher, &node);
		if (node_ok)     rcl_node_fini(&node);
		if (support_ok)  rclc_support_fini(&support);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}
