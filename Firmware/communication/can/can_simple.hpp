#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "canbus.hpp"
#include "axis.hpp"

class CANSimple {
   public:
    enum {
        MSG_CO_NMT_CTRL = 0x000,  // CANOpen NMT Message REC
        MSG_ODRIVE_HEARTBEAT,
        MSG_ODRIVE_ESTOP,
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_ENCODER_COUNT,
        MSG_SET_CONTROLLER_MODES,
        MSG_SET_INPUT_POS,
        MSG_SET_INPUT_VEL,
        MSG_SET_INPUT_TORQUE,
        MSG_SET_LIMITS,
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_INERTIA,
        MSG_GET_IQ,
        MSG_GET_SENSORLESS_ESTIMATES,
        MSG_RESET_ODRIVE,
        MSG_GET_VBUS_VOLTAGE,
        MSG_CLEAR_ERRORS,
        MSG_SET_LINEAR_COUNT,
        MSG_SET_POS_GAIN,
        MSG_SET_VEL_GAINS,
        MSG_SET_INPUT_GOTO_POS,
        MSG_TRIGGER_GOTO_POS,
        MSG_GET_ADC_VOLTAGE,
        MSG_SET_CALIBRATION_CURRENT,
        MSG_PLACEHOLDER_0,
        MSG_PLACEHOLDER_1,
        MSG_PLACEHOLDER_2,
        MSG_PLACEHOLDER_3,
        MSG_PLACEHOLDER_4,
        MSG_PLACEHOLDER_5,
        MSG_PLACEHOLDER_6,
        MSG_PLACEHOLDER_7,
        MSG_PLACEHOLDER_8,
        MSG_PLACEHOLDER_9,
        MSG_SET_BOARD_CONFIG,
        MSG_SET_AXIS_CONFIG,
        MSG_SET_MOTOR_CONFIG_A,
        MSG_SET_MOTOR_CONFIG_B,
        MSG_SET_MOTOR_CONFIG_C,
        MSG_SET_MOTOR_CONFIG_D,
        MSG_SET_MOTOR_CONFIG_E,
        MSG_SET_MOTOR_CONFIG_F,
        MSG_SET_MOTOR_CONFIG_G,
        MSG_SET_CONTROLLER_CONFIG_A,
        MSG_SET_CONTROLLER_CONFIG_B,
        MSG_SET_CONTROLLER_CONFIG_C,
        MSG_SET_CONTROLLER_CONFIG_D,
        MSG_SET_ENCODER_CONFIG_A,
        MSG_SET_ENCODER_CONFIG_B,
        MSG_SET_ENCODER_CONFIG_C,
        MSG_SET_ENCODER_CONFIG_D,
        MSG_SET_ENCODER_CONFIG_E,

        MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
    };

    CANSimple(CanBusBase* canbus) : canbus_(canbus) {}

    bool init();
    uint32_t service_stack();

   private:

    bool renew_subscription(size_t i);
    bool send_heartbeat(const Axis& axis);

    void handle_can_message(const can_Message_t& msg);

    void do_command(Axis& axis, const can_Message_t& cmd);

    // Get functions (msg.rtr bit must be set)
    bool get_motor_error_callback(const Axis& axis);
    bool get_encoder_error_callback(const Axis& axis);
    bool get_controller_error_callback(const Axis& axis);
    bool get_sensorless_error_callback(const Axis& axis);
    bool get_encoder_estimates_callback(const Axis& axis);
    bool get_encoder_count_callback(const Axis& axis);
    bool get_iq_callback(const Axis& axis);
    bool get_sensorless_estimates_callback(const Axis& axis);
    bool get_vbus_voltage_callback(const Axis& axis);
    bool get_adc_voltage_callback(const Axis& axis);

    // Set functions
    static void set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_startup_config_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_pos_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_vel_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_torque_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_modes_callback(Axis& axis, const can_Message_t& msg);
    static void set_limits_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg);
    static void set_traj_inertia_callback(Axis& axis, const can_Message_t& msg);
    static void set_linear_count_callback(Axis& axis, const can_Message_t& msg);
    static void set_pos_gain_callback(Axis& axis, const can_Message_t& msg);
    static void set_vel_gains_callback(Axis& axis, const can_Message_t& msg);
    static void set_input_goto_pos_callback(Axis& axis, const can_Message_t& msg);
    static void trigger_goto_pos_callback(Axis& axis, const can_Message_t& msg);
    static void set_calibration_current_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_0_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_1_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_2_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_3_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_4_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_5_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_6_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_7_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_8_callback(Axis& axis, const can_Message_t& msg);
    static void set_placeholder_9_callback(Axis& axis, const can_Message_t& msg);
    static void set_board_config_callback(Axis& axis, const can_Message_t& msg);
    static void set_axis_config_callback(Axis& axis, const can_Message_t& msg);
    static void set_motor_config_a_callback(Axis& axis, const can_Message_t& msg);
    static void set_motor_config_b_callback(Axis& axis, const can_Message_t& msg);
    static void set_motor_config_c_callback(Axis& axis, const can_Message_t& msg);
    static void set_motor_config_d_callback(Axis& axis, const can_Message_t& msg);
    static void set_motor_config_e_callback(Axis& axis, const can_Message_t& msg);
    static void set_motor_config_f_callback(Axis& axis, const can_Message_t& msg);
    static void set_motor_config_g_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_config_a_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_config_b_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_config_c_callback(Axis& axis, const can_Message_t& msg);
    static void set_controller_config_d_callback(Axis& axis, const can_Message_t& msg);
    static void set_encoder_config_a_callback(Axis& axis, const can_Message_t& msg);
    static void set_encoder_config_b_callback(Axis& axis, const can_Message_t& msg);
    static void set_encoder_config_c_callback(Axis& axis, const can_Message_t& msg);
    static void set_encoder_config_d_callback(Axis& axis, const can_Message_t& msg);
    static void set_encoder_config_e_callback(Axis& axis, const can_Message_t& msg);

    // Other functions
    static void nmt_callback(const Axis& axis, const can_Message_t& msg);
    static void estop_callback(Axis& axis, const can_Message_t& msg);
    static void clear_errors_callback(Axis& axis, const can_Message_t& msg);
    static void start_anticogging_callback(const Axis& axis, const can_Message_t& msg);

    static constexpr uint8_t NUM_NODE_ID_BITS = 5;
    static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

    // Utility functions
    static constexpr uint32_t get_node_id(uint32_t msgID) {
        return (msgID >> NUM_CMD_ID_BITS);  // Upper 5 or more bits
    };

    static constexpr uint8_t get_cmd_id(uint32_t msgID) {
        return (msgID & 0x03F);  // Bottom 6 bits
    }

    CanBusBase* canbus_;
    CanBusBase::CanSubscription* subscription_handles_[AXIS_COUNT];

    // TODO: we this is a hack but actually we should use protocol hooks to
    // renew our filter when the node ID changes
    uint32_t node_ids_[AXIS_COUNT];
    bool extended_node_ids_[AXIS_COUNT];
};

#endif
