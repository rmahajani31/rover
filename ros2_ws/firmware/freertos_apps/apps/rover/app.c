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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    char _err_buf[STRING_BUFFER_LEN]; \
    snprintf(_err_buf, STRING_BUFFER_LEN, "FAIL line %d: %d", __LINE__, (int)temp_rc); \
    printf("%s\n", _err_buf); \
    if(debug_publisher_ready) { for(int _i=0; _i<20; _i++) { publish_debug(_err_buf); vTaskDelay(pdMS_TO_TICKS(200)); } } \
    vTaskDelete(NULL); }}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

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

// Initialize Publishers
void init_publishers(rcl_node_t *node)
{	
	// Initialize the debug message
	outcoming_debug.data.data = debug_buffer;
	outcoming_debug.data.size = 0;
	outcoming_debug.data.capacity = STRING_BUFFER_LEN;

	RCCHECK(rclc_publisher_init_default(&debug_publisher, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/debug"));
	debug_publisher_ready = true;

	left_whl_msg.data.data = left_whl_buffer;
	left_whl_msg.data.size = 0;
	left_whl_msg.data.capacity = STRING_BUFFER_LEN;
	RCCHECK(rclc_publisher_init_default(&left_whl_publisher, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/left_whl"));

	right_whl_msg.data.data = right_whl_buffer;
	right_whl_msg.data.size = 0;
	right_whl_msg.data.capacity = STRING_BUFFER_LEN;
	RCCHECK(rclc_publisher_init_default(&right_whl_publisher, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/right_whl"));
}

// Initialize Subscribers
void init_subscribers(rcl_node_t *node)
{
	RCCHECK(rclc_subscription_init_best_effort(&cmd_vel_subscriber, node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/cmd_vel"));
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

	// Publish the received command velocity
	char temp_buf[STRING_BUFFER_LEN];
    snprintf(temp_buf, STRING_BUFFER_LEN, "CMD: %.2f, %.2f", linear_x, angular_z);
    publish_debug(temp_buf);

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
	snprintf(temp_buf, STRING_BUFFER_LEN, "L: %.2f, R: %.2f", left_velocity, right_velocity);
    publish_debug(temp_buf);
}


void appMain(void *argument)
{
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	printf("Rover app started\n");

	// Initialize hardware
	printf("Initializing hardware with encoders...\n");
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
	bool in_safety_stop = false;
	last_cmd_vel_time = esp_timer_get_time() / 1000;
	int64_t last_enc_publish_time = esp_timer_get_time() / 1000;

	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

		// --- ENCODER PUBLISH (20Hz) ---
		int64_t now_enc = esp_timer_get_time() / 1000;
		if (now_enc - last_enc_publish_time >= 50) {
			left_whl_msg.data.size  = snprintf(left_whl_buffer,  STRING_BUFFER_LEN, "lf:%d,lr:%d", enc_lf, enc_lr);
			right_whl_msg.data.size = snprintf(right_whl_buffer, STRING_BUFFER_LEN, "rf:%d,rr:%d", enc_rf, enc_rr);
			rcl_publish(&left_whl_publisher,  &left_whl_msg,  NULL);
			rcl_publish(&right_whl_publisher, &right_whl_msg, NULL);
			last_enc_publish_time = now_enc;
		}

		// --- SAFETY STOP LOGIC ---
		int64_t now = esp_timer_get_time() / 1000;
		if(now - last_cmd_vel_time > TIMEOUT_MS) {
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
