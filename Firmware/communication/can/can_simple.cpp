
#include "can_simple.hpp"

#include <odrive_main.h>

bool CANSimple::init() {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!renew_subscription(i)) {
            return false;
        }
    }

    return true;
}

bool CANSimple::renew_subscription(size_t i) {
    Axis& axis = axes[i];

    // TODO: remove these two lines (see comment in header)
    node_ids_[i] = axis.config_.can.node_id;
    extended_node_ids_[i] = axis.config_.can.is_extended;

    MsgIdFilterSpecs filter = {
        .id = {},
        .mask = (uint32_t)(0xffffffff << NUM_CMD_ID_BITS)};
    if (axis.config_.can.is_extended) {
        filter.id = (uint32_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    } else {
        filter.id = (uint16_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    }

    if (subscription_handles_[i]) {
        canbus_->unsubscribe(subscription_handles_[i]);
    }

    return canbus_->subscribe(
        filter, [](void* ctx, const can_Message_t& msg) {
            ((CANSimple*)ctx)->handle_can_message(msg);
        },
        this, &subscription_handles_[i]);
}

void CANSimple::handle_can_message(const can_Message_t& msg) {
    //     Frame
    // nodeID | CMD
    // 5 bits | 6 bits
    uint32_t nodeID = get_node_id(msg.id);

    // TODO: board config!!!
    // TODO: watchdog timer
    // TODO change goto

    for (auto& axis : axes) {
        if ((axis.config_.can.node_id == nodeID) && (axis.config_.can.is_extended == msg.isExt)) {
            do_command(axis, msg);
            return;
        }
    }
}

void CANSimple::do_command(Axis& axis, const can_Message_t& msg) {
    const uint32_t cmd = get_cmd_id(msg.id);
    axis.watchdog_feed();
    switch (cmd) {
        case MSG_CO_NMT_CTRL:
            break;
        case MSG_CO_HEARTBEAT_CMD:
            break;
        case MSG_ODRIVE_HEARTBEAT:
            // We don't currently do anything to respond to ODrive heartbeat messages
            break;
        case MSG_ODRIVE_ESTOP:
            estop_callback(axis, msg);
            break;
        case MSG_GET_MOTOR_ERROR:
            if (msg.rtr)
                get_motor_error_callback(axis);
            break;
        case MSG_GET_ENCODER_ERROR:
            if (msg.rtr)
                get_encoder_error_callback(axis);
            break;
        case MSG_GET_SENSORLESS_ERROR:
            if (msg.rtr)
                get_sensorless_error_callback(axis);
            break;
        case MSG_SET_AXIS_NODE_ID:
            set_axis_nodeid_callback(axis, msg);
            break;
        case MSG_SET_AXIS_REQUESTED_STATE:
            set_axis_requested_state_callback(axis, msg);
            break;
        case MSG_SET_AXIS_STARTUP_CONFIG:
            set_axis_startup_config_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_ESTIMATES:
            if (msg.rtr)
                get_encoder_estimates_callback(axis);
            break;
        case MSG_GET_ENCODER_COUNT:
            if (msg.rtr)
                get_encoder_count_callback(axis);
            break;
        case MSG_SET_INPUT_POS:
            set_input_pos_callback(axis, msg);
            break;
        case MSG_SET_INPUT_VEL:
            set_input_vel_callback(axis, msg);
            break;
        case MSG_SET_INPUT_TORQUE:
            set_input_torque_callback(axis, msg);
            break;
        case MSG_SET_CONTROLLER_MODES:
            set_controller_modes_callback(axis, msg);
            break;
        case MSG_SET_LIMITS:
            set_limits_callback(axis, msg);
            break;
        case MSG_START_ANTICOGGING:
            start_anticogging_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_INERTIA:
            set_traj_inertia_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_ACCEL_LIMITS:
            set_traj_accel_limits_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_VEL_LIMIT:
            set_traj_vel_limit_callback(axis, msg);
            break;
        case MSG_GET_IQ:
            if (msg.rtr)
                get_iq_callback(axis);
            break;
        case MSG_GET_SENSORLESS_ESTIMATES:
            if (msg.rtr)
                get_sensorless_estimates_callback(axis);
            break;
        case MSG_RESET_ODRIVE:
            NVIC_SystemReset();
            break;
        case MSG_GET_VBUS_VOLTAGE:
            if (msg.rtr)
                get_vbus_voltage_callback(axis);
            break;
        case MSG_CLEAR_ERRORS:
            clear_errors_callback(axis, msg);
            break;
        case MSG_SET_LINEAR_COUNT:
            set_linear_count_callback(axis, msg);
            break;
        case MSG_SET_POS_GAIN:
            set_pos_gain_callback(axis, msg);
            break;
        case MSG_SET_VEL_GAINS:
            set_vel_gains_callback(axis, msg);
            break;
        case MSG_SET_INPUT_GOTO_POS:
            set_input_goto_pos_callback(axis, msg);
            break;
        case MSG_TRIGGER_GOTO_POS:
            trigger_goto_pos_callback(axis, msg);
            break;
        case MSG_GET_ADC_VOLTAGE:
            if (msg.rtr)
                get_adc_voltage_callback(axis, msg);
            break;
        case MSG_SET_BOARD_CONFIG:
            set_board_config_callback(axis, msg);
            break;
        case MSG_SET_AXIS_CONFIG:
            set_axis_config_callback(axis, msg);
            break;
        case MSG_SET_MOTOR_CONFIG_A:
            set_motor_config_a_callback(axis, msg);
            break;
        case MSG_SET_MOTOR_CONFIG_B:
            set_motor_config_b_callback(axis, msg);
            break;
        case MSG_SET_MOTOR_CONFIG_C:
            set_motor_config_c_callback(axis, msg);
            break;
        case MSG_SET_MOTOR_CONFIG_D:
            set_motor_config_d_callback(axis, msg);
            break;
        case MSG_SET_MOTOR_CONFIG_E:
            set_motor_config_e_callback(axis, msg);
            break;
        case MSG_SET_MOTOR_CONFIG_F:
            set_motor_config_f_callback(axis, msg);
            break;
        case MSG_SET_MOTOR_CONFIG_G:
            set_motor_config_g_callback(axis, msg);
            break;
        case MSG_SET_CONTROLLER_CONFIG_A:
            set_controller_config_a_callback(axis, msg);
            break;
        case MSG_SET_CONTROLLER_CONFIG_B:
            set_controller_config_b_callback(axis, msg);
            break;
        case MSG_SET_CONTROLLER_CONFIG_C:
            set_controller_config_c_callback(axis, msg);
            break;
        case MSG_SET_CONTROLLER_CONFIG_D:
            set_controller_config_d_callback(axis, msg);
            break;
        case MSG_SET_ENCODER_CONFIG_A:
            set_encoder_config_a_callback(axis, msg);
            break;
        case MSG_SET_ENCODER_CONFIG_B:
            set_encoder_config_b_callback(axis, msg);
            break;
        case MSG_SET_ENCODER_CONFIG_C:
            set_encoder_config_c_callback(axis, msg);
            break;
        case MSG_SET_ENCODER_CONFIG_D:
            set_encoder_config_d_callback(axis, msg);
            break;
        case MSG_SET_ENCODER_CONFIG_E:
            set_encoder_config_e_callback(axis, msg);
            break;
        default:
            break;
    }
}

void CANSimple::nmt_callback(const Axis& axis, const can_Message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis& axis, const can_Message_t& msg) {
    axis.error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

bool CANSimple::get_motor_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_MOTOR_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.motor_.error_, 0, 64, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_encoder_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.encoder_.error_, 0, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_sensorless_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_SENSORLESS_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.sensorless_estimator_.error_, 0, 32, true);

    return canbus_->send_message(txmsg);
}

void CANSimple::set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg) {
    axis.config_.can.node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void CANSimple::set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg) {
    axis.requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 16, true));
}

void CANSimple::set_axis_startup_config_callback(Axis& axis, const can_Message_t& msg) {
    // Not Implemented
}

bool CANSimple::get_encoder_estimates_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_ESTIMATES;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal<float>(txmsg, axis.encoder_.pos_estimate_.any().value_or(0.0f), 0, 32, true);
    can_setSignal<float>(txmsg, axis.encoder_.vel_estimate_.any().value_or(0.0f), 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_sensorless_estimates_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_SENSORLESS_ESTIMATES;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    static_assert(sizeof(float) == sizeof(axis.sensorless_estimator_.pll_pos_));

    can_setSignal<float>(txmsg, axis.sensorless_estimator_.pll_pos_, 0, 32, true);
    can_setSignal<float>(txmsg, axis.sensorless_estimator_.vel_estimate_.any().value_or(0.0f), 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_encoder_count_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_COUNT;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal<int32_t>(txmsg, axis.encoder_.shadow_count_, 0, 32, true);
    can_setSignal<int32_t>(txmsg, axis.encoder_.count_in_cpr_, 32, 32, true);
    return canbus_->send_message(txmsg);
}

void CANSimple::set_input_pos_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_pos_ = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
    axis.controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
    axis.controller_.input_pos_updated();
}

void CANSimple::set_input_vel_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_input_torque_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_controller_modes_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.control_mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
    axis.controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
}

void CANSimple::set_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.current_lim = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::start_anticogging_callback(const Axis& axis, const can_Message_t& msg) {
    axis.controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_traj_inertia_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_linear_count_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.set_linear_count(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimple::set_pos_gain_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.pos_gain = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_vel_gains_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_input_goto_pos_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_pos_goto_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::trigger_goto_pos_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_pos_ = axis.controller_.input_pos_goto_;
    axis.controller_.input_pos_updated();
}

void CANSimple::set_board_config_callback(Axis& axis, const can_Message_t& msg) {
    // TODO check if this is correct

    uint8_t board_config_flags = can_getSignal<uint8_t>(msg, 0, 8, true);
    odrv.config_.enable_brake_resistor = (board_config_flags & (1<<7)) != 0;

    odrv.config_.brake_resistance = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_axis_config_callback(Axis& axis, const can_Message_t& msg) {
    uint8_t axisConfigFlags = can_getSignal<uint8_t>(msg, 0, 8, true);
    axis.config_.startup_motor_calibration = (axisConfigFlags & (1<<7)) != 0;
    axis.config_.startup_encoder_index_search = (axisConfigFlags & (1<<6)) != 0;
    axis.config_.startup_encoder_offset_calibration = (axisConfigFlags & (1<<5)) != 0;
    axis.config_.startup_closed_loop_control = (axisConfigFlags & (1<<4)) != 0;
    axis.config_.startup_homing = (axisConfigFlags & (1<<3)) != 0;
    axis.config_.enable_step_dir = (axisConfigFlags & (1<<2)) != 0;
    axis.config_.step_dir_always_on = (axisConfigFlags & (1<<1)) != 0;
    axis.config_.enable_sensorless_mode = (axisConfigFlags & 1) != 0;

    uint8_t moreAxisConfigFlags = can_getSignal<uint8_t>(msg, 8, 8, true);
    axis.config_.enable_watchdog = (moreAxisConfigFlags & (1<<7)) != 0;

    axis.config_.watchdog_timeout = can_getSignal<float>(msg, 16, 32, true);

    axis.config_.step_gpio_pin = uint16_t(can_getSignal<uint8_t>(msg, 48, 8, true));  // should be uint16_t but value will never be that high anyway...
    axis.config_.dir_gpio_pin = uint16_t(can_getSignal<uint8_t>(msg, 56, 8, true));   // same as above
}

void CANSimple::set_motor_config_a_callback(Axis& axis, const can_Message_t& msg) {
    uint8_t motorConfigFlags = can_getSignal<uint8_t>(msg, 0, 8, true);
    axis.motor_.config_.pre_calibrated = (motorConfigFlags & (1<<7)) != 0;

    axis.motor_.config_.pole_pairs = can_getSignal<int32_t>(msg, 8, 32, true);
}

void CANSimple::set_motor_config_b_callback(Axis& axis, const can_Message_t& msg) {
    axis.motor_.config_.calibration_current = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.resistance_calib_max_voltage = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_motor_config_c_callback(Axis& axis, const can_Message_t& msg) {
    axis.motor_.config_.phase_inductance = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.phase_resistance = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_motor_config_d_callback(Axis& axis, const can_Message_t& msg) {
    axis.motor_.config_.torque_constant = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.motor_type = ODriveIntf::MotorIntf::MotorType(can_getSignal<uint8_t>(msg, 32, 8, true));
}

void CANSimple::set_motor_config_e_callback(Axis& axis, const can_Message_t& msg) {
    axis.motor_.config_.current_lim = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.current_lim_margin = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_motor_config_f_callback(Axis& axis, const can_Message_t& msg) {
    axis.motor_.config_.requested_current_range = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.inverter_temp_limit_lower = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_motor_config_g_callback(Axis& axis, const can_Message_t& msg) {
    axis.motor_.config_.inverter_temp_limit_upper = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.current_control_bandwidth = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_controller_config_a_callback(Axis& axis, const can_Message_t& msg) {
    uint8_t controllerConfigFlags = can_getSignal<uint8_t>(msg, 0, 8, true);
    axis.controller_.config_.enable_vel_limit = (controllerConfigFlags & (1<<7)) != 0;
    axis.controller_.config_.enable_torque_mode_vel_limit = (controllerConfigFlags & (1<<6)) != 0;
    axis.controller_.config_.enable_gain_scheduling = (controllerConfigFlags & (1<<5)) != 0;
    axis.controller_.config_.enable_overspeed_error = (controllerConfigFlags & (1<<4)) != 0;

    axis.controller_.config_.control_mode = ODriveIntf::ControllerIntf::ControlMode(can_getSignal<uint8_t>(msg, 8, 8, true));
    axis.controller_.config_.input_mode = ODriveIntf::ControllerIntf::InputMode(can_getSignal<uint8_t>(msg, 16, 8, true));
    axis.controller_.config_.pos_gain = can_getSignal<float>(msg, 24, 32, true);
}

void CANSimple::set_controller_config_b_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_controller_config_c_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.config_.vel_limit_tolerance = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_controller_config_d_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_ramp_rate = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.config_.torque_ramp_rate = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_encoder_config_a_callback(Axis& axis, const can_Message_t& msg) {
    uint8_t encoderConfigFlags = can_getSignal<uint8_t>(msg, 0, 8, true);
    axis.encoder_.config_.use_index = (encoderConfigFlags & (1<<7)) != 0;
    axis.encoder_.config_.use_index_offset = (encoderConfigFlags & (1<<6)) != 0;
    axis.encoder_.config_.find_idx_on_lockin_only = (encoderConfigFlags & (1<<5)) != 0;
    axis.encoder_.config_.pre_calibrated = (encoderConfigFlags & (1<<4)) != 0;
    axis.encoder_.config_.enable_phase_interpolation = (encoderConfigFlags & (1<<3)) != 0;

    axis.encoder_.config_.mode = ODriveIntf::EncoderIntf::Mode(can_getSignal<uint8_t>(msg, 8, 8, true));
    axis.encoder_.config_.index_offset = can_getSignal<float>(msg, 16, 32, true);
}

void CANSimple::set_encoder_config_b_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.config_.cpr = can_getSignal<int32_t>(msg, 0, 32, true);
    axis.encoder_.config_.phase_offset = can_getSignal<int32_t>(msg, 32, 32, true);
}

void CANSimple::set_encoder_config_c_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.config_.phase_offset_float = can_getSignal<float>(msg, 0, 32, true);
    axis.encoder_.config_.direction = can_getSignal<int32_t>(msg, 32, 32, true);
}

void CANSimple::set_encoder_config_d_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.config_.bandwidth = can_getSignal<int32_t>(msg, 0, 32, true);
    axis.encoder_.config_.calib_range = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_encoder_config_e_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.config_.calib_scan_distance = can_getSignal<float>(msg, 0, 32, true);
    axis.encoder_.config_.calib_scan_omega = can_getSignal<float>(msg, 32, 32, true);
}

bool CANSimple::get_iq_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_IQ;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    std::optional<float2D> Idq_setpoint = axis.motor_.current_control_.Idq_setpoint_;
    if (!Idq_setpoint.has_value()) {
        Idq_setpoint = {0.0f, 0.0f};
    }

    static_assert(sizeof(float) == sizeof(Idq_setpoint->first));
    static_assert(sizeof(float) == sizeof(Idq_setpoint->second));
    can_setSignal<float>(txmsg, Idq_setpoint->first, 0, 32, true);
    can_setSignal<float>(txmsg, Idq_setpoint->second, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_vbus_voltage_callback(const Axis& axis) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_VBUS_VOLTAGE;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    uint32_t floatBytes;
    static_assert(sizeof(vbus_voltage) == sizeof(floatBytes));
    can_setSignal<float>(txmsg, vbus_voltage, 0, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_adc_voltage_callback(const Axis& axis) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ADC_VOLTAGE;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    float adc_voltage = odrv.get_adc_voltage(1);
    uint32_t floatBytes;
    static_assert(sizeof(adc_voltage) == sizeof(floatBytes));
    can_setSignal<float>(txmsg, adc_voltage, 0, 32, true);

    return canbus_->send_message(txmsg);
}



void CANSimple::clear_errors_callback(Axis& axis, const can_Message_t& msg) {
    odrv.clear_errors();  // TODO: might want to clear axis errors only
}

uint32_t CANSimple::service_stack() {
    uint32_t nextServiceTime = UINT32_MAX;
    uint32_t now = HAL_GetTick();

    // TODO: remove this polling loop and replace with protocol hook
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        bool node_id_changed = (axes[i].config_.can.node_id != node_ids_[i]) || (axes[i].config_.can.is_extended != extended_node_ids_[i]);
        if (node_id_changed) {
            renew_subscription(i);
        }
    }

    for (auto& a : axes) {
        MEASURE_TIME(a.task_times_.can_heartbeat) {
            if (a.config_.can.heartbeat_rate_ms > 0) {
                if ((now - a.can_.last_heartbeat) >= a.config_.can.heartbeat_rate_ms) {
                    if (send_heartbeat(a))
                        a.can_.last_heartbeat = now;
                }

                int nextAxisService = a.can_.last_heartbeat + a.config_.can.heartbeat_rate_ms - now;
                nextServiceTime = std::min(nextServiceTime, static_cast<uint32_t>(std::max(0, nextAxisService)));
            }

            if (a.config_.can.encoder_rate_ms > 0) {
                if ((now - a.can_.last_encoder) >= a.config_.can.encoder_rate_ms) {
                    if (get_encoder_estimates_callback(a))
                        a.can_.last_encoder = now;
                }

                int nextAxisService = a.can_.last_encoder + a.config_.can.encoder_rate_ms - now;
                nextServiceTime = std::min(nextServiceTime, static_cast<uint32_t>(std::max(0, nextAxisService)));
            }
        }
    }

    return nextServiceTime;
}

bool CANSimple::send_heartbeat(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.error_, 0, 32, true);
    can_setSignal(txmsg, uint8_t(axis.current_state_), 32, 8, true);

    // Motor flags
    uint8_t motorFlags = 0;  // reserved

    // Encoder flags
    uint8_t encoderFlags = 0;  // reserved

    // Controller flags
    uint8_t controllerFlags = 0;
    uint8_t trajDone = uint8_t(axis.controller_.trajectory_done_) << 7;
    controllerFlags |= trajDone;

    can_setSignal(txmsg, motorFlags, 40, 8, true);
    can_setSignal(txmsg, encoderFlags, 48, 8, true);
    can_setSignal(txmsg, controllerFlags, 56, 8, true);
    // can_setSignal(txmsg, axis.current_state_, 32, 32, true);

    return canbus_->send_message(txmsg);
}
