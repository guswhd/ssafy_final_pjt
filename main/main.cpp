#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "driver/uart.h"

//components
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "BNO08x.hpp"

//ROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

static const constexpr char *TAG = "Main";

//ROS domain
#define ROS_DOMAIN_ID 77
static size_t uart_port = UART_NUM_0;

//MOTOR CONFIGURATIONS
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             15000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks

#define MOTOR_LEFT_MCPWM_GPIO_A              17
#define MOTOR_LEFT_MCPWM_GPIO_B              18
#define MOTOR_RIGHT_MCPWM_GPIO_A              15
#define MOTOR_RIGHT_MCPWM_GPIO_B              16

//ENCODER CONFIGURATIONS
#define MOTOR_LEFT_ENC_CHAN_GPIO_A  6
#define MOTOR_LEFT_ENC_CHAN_GPIO_B  7
#define MOTOR_RIGHT_ENC_CHAN_GPIO_A  5
#define MOTOR_RIGHT_ENC_CHAN_GPIO_B  4

#define ENC_HIGH_LIMIT 30000
#define ENC_LOW_LIMIT -30000

//Test for mid-air PWM test
//Right is far from ESP
#define MOTOR_LEFT_MIN_PWM 320//305//270
#define MOTOR_RIGHT_MIN_PWM 327//312//275

#define BDC_PID_LOOP_PERIOD_MS        10   // calculate the motor speed every 10ms
#define BDC_PID_EXPECT_SPEED          570  // expected motor speed, in the pulses counted by the rotary encoder

#define VELOCITY_FILTER_ALPHA 0.4

//Conversion constants
#define CMD_VEL_TO_TICKS_FACTOR 50.0f
#define CMD_TURN_TO_RATE_FACTOR 1.0f

//Fix this value after reinfocing the chassis
#define PITCH_OFFSET -1.8
//-3.0
//ROS Configs

typedef struct {
    bdc_motor_handle_t motor_left;
    bdc_motor_handle_t motor_right;

    pcnt_unit_handle_t pcnt_encoder_left;
    pcnt_unit_handle_t pcnt_encoder_right;

    pid_ctrl_block_handle_t pid_ctrl_balance;
    pid_ctrl_block_handle_t pid_ctrl_velocity;
    pid_ctrl_block_handle_t pid_ctrl_turn;
 
    volatile float target_speed;
    volatile float target_turn_rate;
    
    float lpf_speed;
    float current_pitch;
    float current_yaw_rate; //read from IMU
} robot_control_context_t;

static robot_control_context_t robot_ctrl_ctx;
//============================================ uROS Setups ==================================================
//uROS helper functions
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

static int32_t callback_count = 0;

rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist send_msg;
geometry_msgs__msg__Twist recv_msg;

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    (void) last_call_time;
    if (timer != NULL){
        send_msg.angular.z = robot_ctrl_ctx.current_yaw_rate;
        send_msg.angular.y = robot_ctrl_ctx.current_pitch;
        send_msg.linear.x = robot_ctrl_ctx.target_speed;
        RCSOFTCHECK(rcl_publish(&publisher, &send_msg, NULL));
    }
}

void subscription_callback(const void* msgin){
    callback_count++;
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *) msgin;

    robot_ctrl_ctx.target_speed= (float) msg->linear.x * CMD_VEL_TO_TICKS_FACTOR;
    robot_ctrl_ctx.target_turn_rate = (float) msg->angular.z * CMD_TURN_TO_RATE_FACTOR;
}

void micro_ros_task(void * arg){
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options.
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    //set DOMAIN ID
    rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID);

	// Setup support structure.
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

	// Create node.
	rcl_node_t node = rcl_get_zero_initialized_node();
	RCCHECK(rclc_node_init_default(&node, "balancing_robot_pub_sub", "", &support));

	// Create publisher.
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"robot_state_publisher"));

	// Create subscriber.
	RCCHECK(rclc_subscription_init_default(
		&subscriber,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"cmd_vel"));

	// Create timer.
	rcl_timer_t timer = rcl_get_zero_initialized_timer();
	const unsigned int timer_timeout = 2000;
    RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));
	// RCCHECK(rclc_timer_init_default2(
	// 	&timer,
	// 	&support,
	// 	RCL_MS_TO_NS(timer_timeout),
	// 	timer_callback,
	// 	true));

	// Create executor.
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	unsigned int rcl_wait_timeout = 1000;   // in ms
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor.
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
	RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &recv_msg, &subscription_callback, ON_NEW_DATA));

	// Spin forever.
    geometry_msgs__msg__Twist__init(&send_msg);
    send_msg.angular.z = 0.0f;
    send_msg.angular.y = 0.0f;
    send_msg.linear.x = 0.0f;

	while(1){
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
		usleep(10000);
	}

	// Free resources.
	RCCHECK(rcl_subscription_fini(&subscriber, &node));
	RCCHECK(rcl_publisher_fini(&publisher, &node));
	RCCHECK(rcl_node_fini(&node));

  	vTaskDelete(NULL);
}

//============================================================================================================
// Helper to handle direction switching
void set_motor_speed_safe(bdc_motor_handle_t motor, int speed) {
    if (speed > 0) {
        bdc_motor_forward(motor);
        bdc_motor_set_speed(motor, (uint32_t)speed);
    } else {
        bdc_motor_reverse(motor);
        bdc_motor_set_speed(motor, (uint32_t)(-speed)); // Make positive for API
    }
}

int add_deadzone(float pwm_input, int min_pwm) {
    int final_pwm = 0;

    if (pwm_input > 0.1) {
        final_pwm = (int)(pwm_input + min_pwm);
    } 
    else if (pwm_input < -0.1) {
        final_pwm = (int)(pwm_input - min_pwm);
    } 
    else {
        return 0;
    }
    // --- CRITICAL FIX: SAFETY CLAMP ---
    // Prevent the value from exceeding the hardware limit (400)
    if (final_pwm > BDC_MCPWM_DUTY_TICK_MAX) {
        final_pwm = BDC_MCPWM_DUTY_TICK_MAX;
    }
    else if (final_pwm < -BDC_MCPWM_DUTY_TICK_MAX) {
        final_pwm = -BDC_MCPWM_DUTY_TICK_MAX;
    }

    return final_pwm;
}

// static void pid_loop_cb(void *args)
// {
//     robot_control_context_t *ctx = (robot_control_context_t *)args;
    
//     int left_count = 0;
//     int right_count = 0;    
//     pcnt_unit_get_count(ctx->pcnt_encoder_left, &left_count);
//     pcnt_unit_get_count(ctx->pcnt_encoder_right, &right_count);

//     static int prev_left = 0;
//     static int prev_right = 0;

//     float raw_speed = ((left_count - prev_left) + (right_count - prev_right)) / 2.0f;
//     prev_left = left_count;
//     prev_right = right_count;
    
//     ctx->lpf_speed = (VELOCITY_FILTER_ALPHA * raw_speed) + ((1.0 - VELOCITY_FILTER_ALPHA) *ctx->lpf_speed);

//     //add Cutoff
//     if (fabs(ctx->current_pitch) > 45.0) {
//         set_motor_speed_safe(ctx->motor_left, 0);
//         set_motor_speed_safe(ctx->motor_right, 0);
//         ESP_LOGE(TAG, "Error: Robot fallen, ending PID loop");
//         return;
//     }

//     float target_pitch = 0;
//     //float vel_error = ctx->target_speed - raw_speed;
//     // float velocity_error = ctx->target_speed - ctx->lpf_speed;
//     // pid_compute(ctx->pid_ctrl_velocity, velocity_error, &target_pitch);

//     float balance_pwm = 0;
//     float balance_error = (ctx->current_pitch - PITCH_OFFSET) - target_pitch;
//     //float balance_error = target_pitch - ctx->current_pitch;
//     pid_compute(ctx->pid_ctrl_balance, balance_error, &balance_pwm);

//     float turn_pwm = 0;
//     // float turn_error = ctx->target_turn_rate - ctx->current_yaw_rate;
//     // pid_compute(ctx->pid_ctrl_turn, turn_error, &turn_pwm);

//     float left_total = (int) (balance_pwm + turn_pwm);
//     float right_total = (int) (balance_pwm - turn_pwm);

//     int left_pwm_output = add_deadzone(left_total, MOTOR_LEFT_MIN_PWM);
//     int right_pwm_output = add_deadzone(right_total, MOTOR_RIGHT_MIN_PWM);

//     set_motor_speed_safe(ctx->motor_left, left_pwm_output);
//     set_motor_speed_safe(ctx->motor_right, right_pwm_output);
// }

void update_motor_control(robot_control_context_t *ctx) {
    // robot_control_context_t *ctx = (robot_control_context_t *)ctx;
    
    int left_count = 0;
    int right_count = 0;    
    pcnt_unit_get_count(ctx->pcnt_encoder_left, &left_count);
    pcnt_unit_get_count(ctx->pcnt_encoder_right, &right_count);

    static int prev_left = 0;
    static int prev_right = 0;

    float raw_speed = ((left_count - prev_left) + (right_count - prev_right)) / 2.0f;
    prev_left = left_count;
    prev_right = right_count;
    
    ctx->lpf_speed = (VELOCITY_FILTER_ALPHA * raw_speed) + ((1.0 - VELOCITY_FILTER_ALPHA) *ctx->lpf_speed);

    //add Cutoff
    if (fabs(ctx->current_pitch) > 45.0) {
        set_motor_speed_safe(ctx->motor_left, 0);
        set_motor_speed_safe(ctx->motor_right, 0);
        ESP_LOGE(TAG, "Error: Robot fallen, ending PID loop");
        return;
    }

    float target_pitch = 0;
    //float vel_error = ctx->target_speed - raw_speed;
    float velocity_error = ctx->target_speed - ctx->lpf_speed;
    //float velocity_error = ctx->lpf_speed - ctx->target_speed;
    pid_compute(ctx->pid_ctrl_velocity, velocity_error, &target_pitch);

    float balance_pwm = 0;
    float balance_error = (ctx->current_pitch - PITCH_OFFSET) - target_pitch;
    //float balance_error = target_pitch - ctx->current_pitch;
    pid_compute(ctx->pid_ctrl_balance, balance_error, &balance_pwm);

    float turn_pwm = 0;
    // float turn_error = ctx->target_turn_rate - ctx->current_yaw_rate;
    // pid_compute(ctx->pid_ctrl_turn, turn_error, &turn_pwm);

    float left_total = (int) (balance_pwm + turn_pwm);
    float right_total = (int) (balance_pwm - turn_pwm);

    int left_pwm_output = add_deadzone(left_total, MOTOR_LEFT_MIN_PWM);
    int right_pwm_output = add_deadzone(right_total, MOTOR_RIGHT_MIN_PWM);

    set_motor_speed_safe(ctx->motor_left, left_pwm_output);
    set_motor_speed_safe(ctx->motor_right, right_pwm_output);
}

extern "C" void app_main(void)
{
    robot_ctrl_ctx = {
        .pcnt_encoder_left = NULL,
        .pcnt_encoder_right = NULL,
        .target_speed = 0.0f,
        .current_pitch = 0.0f,
    };
    //==================================== uROS Configuration Starts ===========================================
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

    //==================================== Motor Configuration Starts ===========================================
    //common configs
    bdc_motor_mcpwm_config_t mcpwm_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_config_t motor_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
    };

    //Left Motor
    ESP_LOGI(TAG, "Create Left DC motor");
    bdc_motor_handle_t motor_left = NULL;

    motor_config.pwma_gpio_num = MOTOR_LEFT_MCPWM_GPIO_A;
    motor_config.pwmb_gpio_num = MOTOR_LEFT_MCPWM_GPIO_B;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor_left));
    robot_ctrl_ctx.motor_left = motor_left;
    
    //Right Motor
    ESP_LOGI(TAG, "Create Right DC motor");
    bdc_motor_handle_t motor_right = NULL;
    
    motor_config.pwma_gpio_num = MOTOR_RIGHT_MCPWM_GPIO_A;
    motor_config.pwmb_gpio_num = MOTOR_RIGHT_MCPWM_GPIO_B;
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_config, &mcpwm_config, &motor_right));
    robot_ctrl_ctx.motor_right = motor_right;
    
    //==================================== Encoder Configuration Starts ===========================================
    //Common Configs
    pcnt_unit_config_t unit_config = {
        .low_limit = ENC_LOW_LIMIT,
        .high_limit = ENC_HIGH_LIMIT,
        .flags = {
            .accum_count = true,
        }, // enable counter accumulation
    };
    
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    
    //Left Encoder
    ESP_LOGI(TAG, "Init left pcnt driver to decode rotary signal");
    pcnt_unit_handle_t ml_pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &ml_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(ml_pcnt_unit, &filter_config));

    pcnt_chan_config_t ml_chan_a_config = {
        .edge_gpio_num = MOTOR_LEFT_ENC_CHAN_GPIO_A,
        .level_gpio_num = MOTOR_LEFT_ENC_CHAN_GPIO_B,
    };
    pcnt_chan_config_t ml_chan_b_config = {
        .edge_gpio_num = MOTOR_LEFT_ENC_CHAN_GPIO_B,
        .level_gpio_num = MOTOR_LEFT_ENC_CHAN_GPIO_A,
    };

    pcnt_channel_handle_t ml_pcnt_chan_a = NULL;
    pcnt_channel_handle_t ml_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(ml_pcnt_unit, &ml_chan_a_config, &ml_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(ml_pcnt_unit, &ml_chan_b_config, &ml_pcnt_chan_b));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ml_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ml_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(ml_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(ml_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ml_pcnt_unit, ENC_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(ml_pcnt_unit, ENC_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(ml_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(ml_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(ml_pcnt_unit));
    robot_ctrl_ctx.pcnt_encoder_left = ml_pcnt_unit;
    
    //Right Encoder
    ESP_LOGI(TAG, "Init right pcnt driver to decode rotary signal");
    pcnt_unit_handle_t mr_pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &mr_pcnt_unit));

    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(mr_pcnt_unit, &filter_config));
    pcnt_chan_config_t mr_chan_a_config = {
        .edge_gpio_num = MOTOR_RIGHT_ENC_CHAN_GPIO_A,
        .level_gpio_num = MOTOR_RIGHT_ENC_CHAN_GPIO_B,
    };
    pcnt_chan_config_t mr_chan_b_config = {
        .edge_gpio_num = MOTOR_RIGHT_ENC_CHAN_GPIO_B,
        .level_gpio_num = MOTOR_RIGHT_ENC_CHAN_GPIO_A,
    };
    pcnt_channel_handle_t mr_pcnt_chan_a = NULL;
    pcnt_channel_handle_t mr_pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(mr_pcnt_unit, &mr_chan_a_config, &mr_pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(mr_pcnt_unit, &mr_chan_b_config, &mr_pcnt_chan_b));
    
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(mr_pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(mr_pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(mr_pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(mr_pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(mr_pcnt_unit, ENC_HIGH_LIMIT));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(mr_pcnt_unit, ENC_LOW_LIMIT));

    ESP_ERROR_CHECK(pcnt_unit_enable(mr_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(mr_pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(mr_pcnt_unit));
    robot_ctrl_ctx.pcnt_encoder_right = mr_pcnt_unit;
    
    //==================================== Motor Configuration Ends ===========================================
    //==================================== PID Configuration Starts ===========================================
    //balancing PID
    ESP_LOGI(TAG, "Create PID control block");
    pid_ctrl_parameter_t bal_pid_runtime_param = {
        .kp = 30.0,
        .ki = 0.0,
        .kd = 30.0,
        .max_output   = BDC_MCPWM_DUTY_TICK_MAX - 1,
        .min_output   = -BDC_MCPWM_DUTY_TICK_MAX,
        .max_integral = 200,
        .min_integral = -200,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
    };
    pid_ctrl_block_handle_t bal_pid_ctrl = NULL;
    pid_ctrl_config_t bal_pid_config = {
        .init_param = bal_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&bal_pid_config, &bal_pid_ctrl));
    robot_ctrl_ctx.pid_ctrl_balance = bal_pid_ctrl;
    
    //Velocity PID
    pid_ctrl_parameter_t vel_pid_runtime_param = {
        .kp = 0.01,
        .ki = 0.00,
        .kd = 0.0,
        .max_output   = 10.0,
        .min_output   = -10.0,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
    };
    pid_ctrl_block_handle_t vel_pid_ctrl = NULL;
    pid_ctrl_config_t vel_pid_config = {
        .init_param = vel_pid_runtime_param,
    };
    ESP_ERROR_CHECK(pid_new_control_block(&vel_pid_config, &vel_pid_ctrl));
    robot_ctrl_ctx.pid_ctrl_velocity = vel_pid_ctrl;

    pid_ctrl_parameter_t turn_pid_params = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .max_output = 200,
        .min_output = -200,
        .cal_type = PID_CAL_TYPE_POSITIONAL,
    };
    pid_ctrl_block_handle_t turn_pid_ctrl = NULL;
    pid_ctrl_config_t turn_pid_config = {
        .init_param = turn_pid_params
    };
    ESP_ERROR_CHECK(pid_new_control_block(&turn_pid_config, &turn_pid_ctrl));
    robot_ctrl_ctx.pid_ctrl_turn = turn_pid_ctrl;

    // ESP_LOGI(TAG, "Create a timer to do PID calculation periodically");
    // const esp_timer_create_args_t periodic_timer_args = {
    //     .callback = pid_loop_cb,
    //     .arg = &robot_ctrl_ctx,
    //     .name = "pid_loop"
    // };
    // esp_timer_handle_t pid_loop_timer = NULL;
    // ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &pid_loop_timer));
    
    //==================================== PID Configuration Ends   ===========================================
    ESP_LOGI(TAG, "Enable motor");
    ESP_ERROR_CHECK(bdc_motor_enable(motor_left));
    ESP_ERROR_CHECK(bdc_motor_enable(motor_right));

    // ESP_LOGI(TAG, "Start motor speed loop");
    // ESP_ERROR_CHECK(esp_timer_start_periodic(pid_loop_timer, BDC_PID_LOOP_PERIOD_MS * 1000));
    //Start ROS Loop using RTOS
    xTaskCreatePinnedToCore(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_APP_STACK,
            NULL,
            CONFIG_MICRO_ROS_APP_TASK_PRIO,
            NULL,
            0); //run at core 0, priority 5

    //==================================== IMU Configuration Starts ===========================================
    bno08x_config_t imu_config; //create config struct
    imu_config.io_rst = GPIO_NUM_9;
    imu_config.io_int = GPIO_NUM_8;
    imu_config.io_cs = GPIO_NUM_10;
    imu_config.io_mosi = GPIO_NUM_11;   //assign pin
    imu_config.io_miso = GPIO_NUM_13;   //assign pin
    imu_config.io_sclk = GPIO_NUM_12;
    //etc...
    static BNO08x imu(imu_config);            //pass config to BNO08x constructor

    if (!imu.initialize())
    {
        ESP_LOGE(TAG, "Init failure, returning from main.");
        return;
    }

    // enable game rotation vector and calibrated gyro reports
    imu.rpt.rv_game.enable(2000UL);  // 100,000us == 100ms report interval
    imu.rpt.cal_gyro.enable(2000UL); // 100,000us == 100ms report interval
    //==================================== Main Loop Starts ===========================================
    int64_t last_log_time = esp_timer_get_time();
    while (1) {
        //vTaskDelay(pdMS_TO_TICKS(1)); // 0.5초마다 출력 (너무 빠르면 보기 힘듭니다)
        // block until new report is detected
        if (imu.data_available())
        {
            // check for game rotation vector report
            if (imu.rpt.rv_game.has_new_data())
            {
                // get absolute heading in degrees
                bno08x_euler_angle_t euler = imu.rpt.rv_game.get_euler();
                // display heading
                robot_ctrl_ctx.current_pitch = euler.x;
                update_motor_control(&robot_ctrl_ctx);
            }

            // check for cal gyro report
            if (imu.rpt.cal_gyro.has_new_data())
            {
                // get angular velocity in rad/s
                bno08x_gyro_t velocity = imu.rpt.cal_gyro.get();
                // display velocity
                robot_ctrl_ctx.current_yaw_rate = velocity.z;
            }
        }
        if (esp_timer_get_time() - last_log_time > 500000){
            last_log_time = esp_timer_get_time();
            ESP_LOGI(TAG, "Pitch: %.2f | Speed: %.2f", 
            robot_ctrl_ctx.current_pitch, 
            robot_ctrl_ctx.lpf_speed);
        }
    }
}