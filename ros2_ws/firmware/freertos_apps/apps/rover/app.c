// Imports
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist_stamped.h>
#include <std_msgs/msg/string.h>

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_timer.h"
#include "esp_system.h"

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
#define TIMEOUT_MS 1000 // Stop motors if no msg for 1 second
#define PWM_FREQUENCY 20000 // Hz
#define MAX_SPEED 1.0f // m/s
#define MAX_ANGULAR_SPEED ((2.0f * MAX_SPEED) / TRACK_WIDTH) // rad/s
#define MOTOR_COMMAND_DEADBAND 0.005f // m/s
#define MIN_ACTIVE_DUTY 45U // Tune for the minimum PWM that moves the rover
#define MAX_DUTY 255U
#define AGENT_PING_TIMEOUT_MS 1000
#define AGENT_RETRY_BACKOFF_MS 500
#define AGENT_SETTLE_DELAY_MS 250
#define MOTOR_EPSILON 0.001f

// Forward declarations
void publish_debug(const char * msg);
rcl_ret_t set_motor_speed(int channel, int dir_pin, float speed);


// Global variables
int64_t last_cmd_vel_time = 0;
bool debug_publisher_ready = false;
float latest_linear_x = 0.0f;
float latest_angular_z = 0.0f;
float applied_left_velocity = 0.0f;
float applied_right_velocity = 0.0f;

typedef enum {
    RUNTIME_PHASE_NONE = 0,
    RUNTIME_PHASE_BOOT = 1,
    RUNTIME_PHASE_CALLBACK_ENTER = 2,
    RUNTIME_PHASE_CALLBACK_STORED = 3,
    RUNTIME_PHASE_RUN_COMPUTE = 4,
    RUNTIME_PHASE_RUN_APPLY_LEFT = 5,
    RUNTIME_PHASE_RUN_APPLY_RIGHT = 6,
    RUNTIME_PHASE_RUN_APPLY_DONE = 7,
    RUNTIME_PHASE_RUN_FLUSH_DEBUG = 8,
    RUNTIME_PHASE_SAFETY_STOP = 9,
    RUNTIME_PHASE_EXECUTOR_ERROR = 10,
    RUNTIME_PHASE_CLEANUP = 11
} runtime_phase_t;

// Kept across resets so the next boot can report where the previous runtime
// stopped, which is useful for watchdog/panic debugging.
static RTC_NOINIT_ATTR int last_runtime_phase = RUNTIME_PHASE_NONE;

// Pre-init log buffer — captures messages before the debug publisher is ready
#define LOG_BUFFER_COUNT 16
static char log_buffer[LOG_BUFFER_COUNT][STRING_BUFFER_LEN];
static int log_buffer_head = 0;
static int current_init_attempt = 0;
static char last_retry_summary[STRING_BUFFER_LEN];
static bool last_retry_summary_pending = false;

#define DEBUG_EVENT_COUNT 24
static char debug_event_buffer[DEBUG_EVENT_COUNT][STRING_BUFFER_LEN];
static int debug_event_head = 0;
static int debug_event_tail = 0;
static bool debug_event_full = false;

static float clampf(float value, float min_value, float max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
	    }
	    return value;
	}

static bool is_valid_runtime_phase(int phase) {
    return phase >= RUNTIME_PHASE_NONE && phase <= RUNTIME_PHASE_CLEANUP;
}

void clear_log_buffer(void) {
    log_buffer_head = 0;
}

void append_truncated_string(char *dest, size_t dest_size, size_t write_index, const char *src) {
    if (write_index >= dest_size) {
        if (dest_size > 0) {
            dest[dest_size - 1] = '\0';
        }
        return;
    }

    size_t max_copy = dest_size - write_index - 1;
    size_t copy_len = strnlen(src, max_copy);
    memcpy(dest + write_index, src, copy_len);
    dest[write_index + copy_len] = '\0';
}

void log_msg(const char *msg) {
    char temp_buf[STRING_BUFFER_LEN];
    const char * message_to_log = msg;
    if (current_init_attempt > 0) {
        int len = snprintf(temp_buf, STRING_BUFFER_LEN, "[attempt %d] %s", current_init_attempt, msg);
        if (len > 0 && len < STRING_BUFFER_LEN) {
            message_to_log = temp_buf;
        }
    }

    if (debug_publisher_ready) {
        publish_debug(message_to_log);
    } else {
        if (log_buffer_head < LOG_BUFFER_COUNT) {
            snprintf(log_buffer[log_buffer_head++], STRING_BUFFER_LEN, "%s", message_to_log);
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

void clear_debug_event_buffer(void) {
    debug_event_head = 0;
    debug_event_tail = 0;
    debug_event_full = false;
}

void queue_debug_event(const char *msg) {
    // Runtime events are buffered so the control loop can report them without
    // blocking on a ROS publish at the exact moment they occur.
    snprintf(debug_event_buffer[debug_event_head], STRING_BUFFER_LEN, "%s", msg);
    debug_event_head = (debug_event_head + 1) % DEBUG_EVENT_COUNT;
    if (debug_event_full) {
        debug_event_tail = (debug_event_tail + 1) % DEBUG_EVENT_COUNT;
    } else if (debug_event_head == debug_event_tail) {
        debug_event_full = true;
    }
}

bool publish_one_debug_event(void) {
    if (!debug_event_full && debug_event_tail == debug_event_head) {
        return false;
    }

    publish_debug(debug_event_buffer[debug_event_tail]);
    debug_event_tail = (debug_event_tail + 1) % DEBUG_EVENT_COUNT;
    debug_event_full = false;
    return true;
}

void stop_all_motors(void) {
    // Used both as a safety stop and during reconnect cleanup.
    set_motor_speed(LEDC_CHANNEL_0, M_LF_DIR, 0.0f);
    set_motor_speed(LEDC_CHANNEL_1, M_LR_DIR, 0.0f);
    set_motor_speed(LEDC_CHANNEL_2, M_RF_DIR, 0.0f);
    set_motor_speed(LEDC_CHANNEL_3, M_RR_DIR, 0.0f);
}

const char * runtime_phase_to_string(int phase) {
    switch (phase) {
        case RUNTIME_PHASE_NONE:           return "NONE";
        case RUNTIME_PHASE_BOOT:           return "BOOT";
        case RUNTIME_PHASE_CALLBACK_ENTER: return "CALLBACK_ENTER";
        case RUNTIME_PHASE_CALLBACK_STORED:return "CALLBACK_STORED";
        case RUNTIME_PHASE_RUN_COMPUTE:    return "RUN_COMPUTE";
        case RUNTIME_PHASE_RUN_APPLY_LEFT: return "RUN_APPLY_LEFT";
        case RUNTIME_PHASE_RUN_APPLY_RIGHT:return "RUN_APPLY_RIGHT";
        case RUNTIME_PHASE_RUN_APPLY_DONE: return "RUN_APPLY_DONE";
        case RUNTIME_PHASE_RUN_FLUSH_DEBUG:return "RUN_FLUSH_DEBUG";
        case RUNTIME_PHASE_SAFETY_STOP:    return "SAFETY_STOP";
        case RUNTIME_PHASE_EXECUTOR_ERROR: return "EXECUTOR_ERROR";
        case RUNTIME_PHASE_CLEANUP:        return "CLEANUP";
        default:                           return "UNKNOWN";
    }
}

const char * reset_reason_to_string(esp_reset_reason_t reason) {
    switch (reason) {
        case ESP_RST_UNKNOWN:   return "UNKNOWN";
        case ESP_RST_POWERON:   return "POWERON";
        case ESP_RST_EXT:       return "EXT";
        case ESP_RST_SW:        return "SW";
        case ESP_RST_PANIC:     return "PANIC";
        case ESP_RST_INT_WDT:   return "INT_WDT";
        case ESP_RST_TASK_WDT:  return "TASK_WDT";
        case ESP_RST_WDT:       return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT:  return "BROWNOUT";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "UNMAPPED";
    }
}

void log_reset_reason(void) {
    char temp_buf[STRING_BUFFER_LEN];
    esp_reset_reason_t reason = esp_reset_reason();
    // Pair the ESP reset reason with the last saved runtime phase to make
    // field failures easier to diagnose from /rover/debug.
    int phase = last_runtime_phase;
    if (!is_valid_runtime_phase(phase)) {
        phase = RUNTIME_PHASE_NONE;
    }
    snprintf(
        temp_buf,
        STRING_BUFFER_LEN,
        "Reset reason: %s (%d)",
        reset_reason_to_string(reason),
        (int) reason
    );
    log_msg(temp_buf);

    snprintf(
        temp_buf,
        STRING_BUFFER_LEN,
        "Last runtime phase: %s (%d)",
        runtime_phase_to_string(phase),
        phase
    );
    log_msg(temp_buf);
    last_runtime_phase = RUNTIME_PHASE_BOOT;
}

void log_rcl_error(const char * context, rcl_ret_t ret_code) {
    char temp_buf[STRING_BUFFER_LEN];
    const char * error_str = rcl_get_error_string().str;
    if (error_str == NULL || error_str[0] == '\0') {
        error_str = "no rcl error string";
    }

    int prefix_len = snprintf(temp_buf, STRING_BUFFER_LEN, "%s (ret=%d): ", context, (int) ret_code);
    if (prefix_len < 0) {
        temp_buf[0] = '\0';
    } else if (prefix_len < STRING_BUFFER_LEN) {
        append_truncated_string(temp_buf, STRING_BUFFER_LEN, (size_t) prefix_len, error_str);
    } else {
        temp_buf[STRING_BUFFER_LEN - 1] = '\0';
	    }
	    log_msg(temp_buf);
	    if (current_init_attempt > 0) {
	        int summary_len = snprintf(
	            last_retry_summary,
	            STRING_BUFFER_LEN,
		            "Attempt %d failed: ",
		            current_init_attempt
		        );
		        if (summary_len < 0) {
		            last_retry_summary[0] = '\0';
		        } else if (summary_len < STRING_BUFFER_LEN) {
		            append_truncated_string(last_retry_summary, STRING_BUFFER_LEN, (size_t) summary_len, temp_buf);
		        } else {
		            last_retry_summary[STRING_BUFFER_LEN - 1] = '\0';
		        }
		        last_retry_summary_pending = true;
		    }
	    rcl_reset_error();
}

rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t debug_publisher;

geometry_msgs__msg__TwistStamped incoming_cmd_vel;
std_msgs__msg__String outcoming_debug;

char debug_buffer[STRING_BUFFER_LEN];


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

    // Configure one shared LEDC timer so all four motor PWM channels run at
    // the same frequency and resolution.
    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // Each wheel gets its own PWM channel while direction is handled by a
    // separate GPIO pin.
    ledc_channel_config_t lcd_ch[4] = {
        {.gpio_num = M_LF_PWM, .channel = LEDC_CHANNEL_0, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = M_LR_PWM, .channel = LEDC_CHANNEL_1, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = M_RF_PWM, .channel = LEDC_CHANNEL_2, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0},
        {.gpio_num = M_RR_PWM, .channel = LEDC_CHANNEL_3, .speed_mode = LEDC_LOW_SPEED_MODE, .timer_sel = LEDC_TIMER_0, .duty = 0}
    };
    for(int i=0; i<4; i++) ledc_channel_config(&lcd_ch[i]);
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
rcl_ret_t set_motor_speed(int channel, int dir_pin, float speed)
{
	// Get magnitude of the speed
	float abs_speed = fabsf(speed);

	// If the command is effectively zero, set the duty cycle to 0.
	if (abs_speed <= MOTOR_COMMAND_DEADBAND) {
        esp_err_t set_ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, 0);
        if (set_ret != ESP_OK) {
            return RCL_RET_ERROR;
        }
        esp_err_t update_ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
        if (update_ret != ESP_OK) {
            return RCL_RET_ERROR;
        }
        return RCL_RET_OK;
    }

	// Convert the requested m/s command into an 8-bit LEDC duty cycle.
    // DC motors need a nonzero PWM floor to overcome static friction, so
    // scale active commands between MIN_ACTIVE_DUTY and MAX_DUTY.
	float speed_ratio = clampf(abs_speed / MAX_SPEED, 0.0f, 1.0f);
	uint32_t duty = MIN_ACTIVE_DUTY + (uint32_t)(speed_ratio * (float)(MAX_DUTY - MIN_ACTIVE_DUTY));
	if (duty > MAX_DUTY) duty = MAX_DUTY;

	// Set direction
	gpio_set_level(dir_pin, (speed >= 0) ? 1 : 0);

	// Update the PWM hardware channel
    esp_err_t set_ret = ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    if (set_ret != ESP_OK) {
        return RCL_RET_ERROR;
    }
    esp_err_t update_ret = ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
    if (update_ret != ESP_OK) {
        return RCL_RET_ERROR;
    }
    return RCL_RET_OK;
}

// Callback function to convert twist message to motor speeds
void cmd_vel_subscription_callback (const void * msgin)
{
	last_runtime_phase = RUNTIME_PHASE_CALLBACK_ENTER;

	// Refresh the last command velocity time
	last_cmd_vel_time = esp_timer_get_time() / 1000;
	
	// Extract the linear and angular velocities from the incoming message
	const geometry_msgs__msg__TwistStamped * msg = (const geometry_msgs__msg__TwistStamped *)msgin;
	// Clamp at the firmware boundary so bad ROS commands cannot exceed the
	// configured motor speed envelope.
	latest_linear_x = clampf(msg->twist.linear.x, -MAX_SPEED, MAX_SPEED);
	latest_angular_z = clampf(msg->twist.angular.z, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED);

	last_runtime_phase = RUNTIME_PHASE_CALLBACK_STORED;
}


void appMain(void *argument)
{
	log_msg("Rover app started");
	log_reset_reason();
	log_msg("Initializing hardware...");
	init_hardware();
	stop_all_motors();
	last_cmd_vel_time = 0;
	latest_linear_x = 0.0f;
	latest_angular_z = 0.0f;
    applied_left_velocity = 0.0f;
    applied_right_velocity = 0.0f;
    clear_debug_event_buffer();
	vTaskDelay(pdMS_TO_TICKS(500));

	typedef enum {
		CONN_WAIT_FOR_AGENT,
		CONN_INIT_MICROROS,
		CONN_RUN_SESSION,
		CONN_CLEANUP
	} connection_state_t;

	connection_state_t state = CONN_WAIT_FOR_AGENT;
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support = {0};
	rcl_node_t node = rcl_get_zero_initialized_node();
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	bool support_ok = false;
	bool node_ok = false;
	bool pub_debug_ok = false;
	bool sub_ok = false;
	bool executor_ok = false;
	bool in_safety_stop = false;
	bool session_should_retry = false;
	int reconnect_attempt = 0;
    int init_attempt = 0;

	while (1) {
		switch (state) {
				case CONN_WAIT_FOR_AGENT: {
					// Stay in a safe stopped state until the Pi-side
					// micro-ROS agent is reachable over serial.
					current_init_attempt = 0;
					debug_publisher_ready = false;
					last_cmd_vel_time = 0;
					in_safety_stop = false;
	                    latest_linear_x = 0.0f;
                    latest_angular_z = 0.0f;
                    applied_left_velocity = 0.0f;
                    applied_right_velocity = 0.0f;
                    clear_debug_event_buffer();
					stop_all_motors();

				log_msg("Agent wait: begin");
				bool agent_found = (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) == RMW_RET_OK);

				if (!agent_found) {
					char temp_buf[STRING_BUFFER_LEN];
					reconnect_attempt++;
					snprintf(
						temp_buf,
						STRING_BUFFER_LEN,
						"Agent wait: unavailable, retrying (%d)",
						reconnect_attempt
					);
					log_msg(temp_buf);
					vTaskDelay(pdMS_TO_TICKS(AGENT_RETRY_BACKOFF_MS));
					break;
					}

	                    init_attempt++;
	                    current_init_attempt = init_attempt;
						log_msg("Agent wait: detected");
                        log_msg("Agent wait: settling transport");
                        vTaskDelay(pdMS_TO_TICKS(AGENT_SETTLE_DELAY_MS));
                        log_msg("Agent wait: transport settled");
						state = CONN_INIT_MICROROS;
						break;
					}

			case CONN_INIT_MICROROS:
				// Build the micro-ROS session from scratch after every
				// reconnect so stale handles are not reused.
				log_msg("support_init: begin");
				rcl_ret_t support_ret = rclc_support_init(&support, 0, NULL, &allocator);
				if (support_ret != RCL_RET_OK) {
					log_rcl_error("support_init failed", support_ret);
					session_should_retry = true;
					state = CONN_CLEANUP;
					break;
				}
				support_ok = true;
				log_msg("support_init: ok");

				log_msg("node_init: begin");
				rcl_ret_t node_ret = rclc_node_init_default(&node, "rover_node", "", &support);
				if (node_ret != RCL_RET_OK) {
					log_rcl_error("node_init failed", node_ret);
					session_should_retry = true;
					state = CONN_CLEANUP;
					break;
				}
				node_ok = true;
				log_msg("node_init: ok");

				outcoming_debug.data.data = debug_buffer;
				outcoming_debug.data.size = 0;
				outcoming_debug.data.capacity = STRING_BUFFER_LEN;
				log_msg("debug_publisher init: begin");
				rcl_ret_t pub_ret = rclc_publisher_init_default(&debug_publisher, &node,
						ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "/rover/debug");
				if (pub_ret != RCL_RET_OK) {
					log_rcl_error("debug_publisher init failed", pub_ret);
					session_should_retry = true;
					state = CONN_CLEANUP;
					break;
				}
				pub_debug_ok = true;
				log_msg("debug_publisher init: ok");

				log_msg("cmd_vel_subscriber init: begin");
				rcl_ret_t sub_ret = rclc_subscription_init_best_effort(&cmd_vel_subscriber, &node,
						ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped), "/cmd_vel");
				if (sub_ret != RCL_RET_OK) {
					log_rcl_error("cmd_vel_subscriber init failed", sub_ret);
					session_should_retry = true;
					state = CONN_CLEANUP;
					break;
				}
				sub_ok = true;
				log_msg("cmd_vel_subscriber init: ok");

				log_msg("executor_init: begin");
				rcl_ret_t executor_ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
				if (executor_ret != RCL_RET_OK) {
					log_rcl_error("executor_init failed", executor_ret);
					session_should_retry = true;
					state = CONN_CLEANUP;
					break;
				}
				executor_ok = true;
				log_msg("executor_init: ok");

				log_msg("executor_add_subscription: begin");
				rcl_ret_t add_sub_ret = rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &incoming_cmd_vel,
						&cmd_vel_subscription_callback, ON_NEW_DATA);
				if (add_sub_ret != RCL_RET_OK) {
					log_rcl_error("executor_add_subscription failed", add_sub_ret);
					session_should_retry = true;
					state = CONN_CLEANUP;
					break;
					}
						debug_publisher_ready = true;
						flush_log_buffer();
                        if (last_retry_summary_pending) {
                            publish_debug(last_retry_summary);
                            last_retry_summary_pending = false;
                        }
						log_msg("executor_add_subscription: ok");

						reconnect_attempt = 0;
						last_cmd_vel_time = 0;
					latest_linear_x = 0.0f;
					latest_angular_z = 0.0f;
                    applied_left_velocity = 0.0f;
                    applied_right_velocity = 0.0f;
					in_safety_stop = false;
					log_msg("micro-ROS fully initialized");
					state = CONN_RUN_SESSION;
					break;

			case CONN_RUN_SESSION: {
				rcl_ret_t spin_ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
				if (spin_ret != RCL_RET_OK && spin_ret != RCL_RET_TIMEOUT) {
					last_runtime_phase = RUNTIME_PHASE_EXECUTOR_ERROR;
                    queue_debug_event("EXECUTOR_ERROR");
					log_rcl_error("executor_spin_some failed", spin_ret);
					session_should_retry = true;
					state = CONN_CLEANUP;
					break;
				}

				if (last_cmd_vel_time > 0) {
                    last_runtime_phase = RUNTIME_PHASE_RUN_COMPUTE;
					// Differential-drive mixing: positive angular_z slows the
					// left side and speeds up the right side.
					float left_velocity = clampf(
						latest_linear_x - (latest_angular_z * TRACK_WIDTH / 2.0f),
						-MAX_SPEED,
						MAX_SPEED
					);
					float right_velocity = clampf(
						latest_linear_x + (latest_angular_z * TRACK_WIDTH / 2.0f),
						-MAX_SPEED,
						MAX_SPEED
					);

                    if (fabsf(left_velocity - applied_left_velocity) > MOTOR_EPSILON ||
                        fabsf(right_velocity - applied_right_velocity) > MOTOR_EPSILON) {
                        // Avoid rewriting LEDC registers for tiny command
                        // changes; this keeps the loop quieter and cheaper.
                        last_runtime_phase = RUNTIME_PHASE_RUN_APPLY_LEFT;
                        rcl_ret_t left_front_ret = set_motor_speed(LEDC_CHANNEL_0, M_LF_DIR, left_velocity);
                        rcl_ret_t left_rear_ret = set_motor_speed(LEDC_CHANNEL_1, M_LR_DIR, left_velocity);
                        if (left_front_ret != RCL_RET_OK || left_rear_ret != RCL_RET_OK) {
                            queue_debug_event("LEDC_LEFT_ERROR");
                            session_should_retry = true;
                            state = CONN_CLEANUP;
                            break;
                        }

                        last_runtime_phase = RUNTIME_PHASE_RUN_APPLY_RIGHT;
                        rcl_ret_t right_front_ret = set_motor_speed(LEDC_CHANNEL_2, M_RF_DIR, right_velocity);
                        rcl_ret_t right_rear_ret = set_motor_speed(LEDC_CHANNEL_3, M_RR_DIR, right_velocity);
                        if (right_front_ret != RCL_RET_OK || right_rear_ret != RCL_RET_OK) {
                            queue_debug_event("LEDC_RIGHT_ERROR");
                            session_should_retry = true;
                            state = CONN_CLEANUP;
                            break;
                        }

                        applied_left_velocity = left_velocity;
                        applied_right_velocity = right_velocity;
                        last_runtime_phase = RUNTIME_PHASE_RUN_APPLY_DONE;
                    }
				}

				int64_t now = esp_timer_get_time() / 1000;
				if (last_cmd_vel_time > 0 && (now - last_cmd_vel_time > TIMEOUT_MS)) {
					if (!in_safety_stop) {
						// If the ROS side stops sending velocity commands,
						// stop once and report the safety event.
						last_runtime_phase = RUNTIME_PHASE_SAFETY_STOP;
						stop_all_motors();
						latest_linear_x = 0.0f;
						latest_angular_z = 0.0f;
                        applied_left_velocity = 0.0f;
                        applied_right_velocity = 0.0f;
                        queue_debug_event("SAFETY STOP - NO CMD_VEL FOR 1000ms");
						in_safety_stop = true;
					}
				} else {
					in_safety_stop = false;
				}

                if (debug_publisher_ready) {
                    last_runtime_phase = RUNTIME_PHASE_RUN_FLUSH_DEBUG;
                    publish_one_debug_event();
                }

				vTaskDelay(pdMS_TO_TICKS(10));
				break;
			}

			case CONN_CLEANUP:
				last_runtime_phase = RUNTIME_PHASE_CLEANUP;
				log_msg("Cleanup: begin");
				stop_all_motors();
				debug_publisher_ready = false;

				if (executor_ok) {
					rclc_executor_fini(&executor);
				}
				if (sub_ok) {
					rcl_subscription_fini(&cmd_vel_subscriber, &node);
				}
				if (pub_debug_ok) {
					rcl_publisher_fini(&debug_publisher, &node);
				}
				if (node_ok) {
					rcl_node_fini(&node);
				}
				if (support_ok) {
					rclc_support_fini(&support);
				}

				executor = rclc_executor_get_zero_initialized_executor();
				node = rcl_get_zero_initialized_node();
				support = (rclc_support_t){0};
				support_ok = false;
				node_ok = false;
				pub_debug_ok = false;
				sub_ok = false;
				executor_ok = false;
				in_safety_stop = false;
				last_cmd_vel_time = 0;
				latest_linear_x = 0.0f;
				latest_angular_z = 0.0f;
                applied_left_velocity = 0.0f;
                applied_right_velocity = 0.0f;

					if (session_should_retry) {
						log_msg("Cleanup: done, will reconnect");
                        clear_log_buffer();
						session_should_retry = false;
					} else {
						log_msg("Cleanup: done");
                        clear_log_buffer();
					}

					vTaskDelay(pdMS_TO_TICKS(AGENT_RETRY_BACKOFF_MS));
					state = CONN_WAIT_FOR_AGENT;
				break;
		}
	}
}
