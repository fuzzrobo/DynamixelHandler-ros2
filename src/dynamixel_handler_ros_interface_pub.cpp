#include "dynamixel_handler.hpp"
#include "myUtils/make_iterator_convenient.hpp"

// 角度変換
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる
double round4(double val) { return round(val*10000.0)/10000.0; }

void DynamixelHandler::BroadcastState_Status(){
    DynamixelStatus msg;
    for (const auto& [id, value] : state_r_) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);

        msg.torque.push_back(tq_mode_[id]==TORQUE_ENABLE);
        msg.error.push_back(has_hardware_err_);
        msg.ping.push_back(ping_err_[id]==0);
        switch(op_mode_[id]) {
            case OPERATING_MODE_CURRENT:              msg.mode.push_back("current");               break;
            case OPERATING_MODE_VELOCITY:             msg.mode.push_back("velocity");              break;
            case OPERATING_MODE_POSITION:             msg.mode.push_back("position");              break;
            case OPERATING_MODE_EXTENDED_POSITION:    msg.mode.push_back("extended_position");     break;
            case OPERATING_MODE_CURRENT_BASE_POSITION:msg.mode.push_back("current_base_position"); break;
        }
    }
    pub_status_->publish(msg);
}

void DynamixelHandler::BroadcastState_Present(){
    DynamixelPresent msg;
    for (const auto& [id, value] : state_r_) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);
        for (auto state : list_read_present_) switch(state) {
            case PRESENT_PWM:          msg.pwm_percent.push_back         (round4(value[state]    )); break;
            case PRESENT_CURRENT:      msg.current_ma.push_back          (round4(value[state]    )); break;
            case PRESENT_VELOCITY:     msg.velocity_deg_s.push_back      (round4(value[state]/DEG)); break;
            case PRESENT_POSITION:     msg.position_deg.push_back        (round4(value[state]/DEG)); break;
            case VELOCITY_TRAJECTORY:  msg.vel_trajectory_deg_s.push_back(round4(value[state]/DEG)); break;
            case POSITION_TRAJECTORY:  msg.pos_trajectory_deg.push_back  (round4(value[state]/DEG)); break;
            case PRESENT_TEMPERATURE:  msg.temperature_degc.push_back    (round4(value[state]    )); break;
            case PRESENT_INPUT_VOLTAGE:msg.input_voltage_v.push_back     (round4(value[state]    )); break;
            default: break;
        }
    }
    pub_present_->publish(msg);
}

void DynamixelHandler::BroadcastState_Error(){
    DynamixelError msg;
    // msg.stamp = this->get_clock()->now();
    for (const auto& [id, error]: hardware_error_) if ( is_in(id, id_set_) ) {
        if (error[INPUT_VOLTAGE     ]) msg.input_voltage.push_back     (id);
        if (error[MOTOR_HALL_SENSOR ]) msg.motor_hall_sensor.push_back (id);
        if (error[OVERHEATING       ]) msg.overheating.push_back       (id);
        if (error[MOTOR_ENCODER     ]) msg.motor_encoder.push_back     (id);
        if (error[ELECTRONICAL_SHOCK]) msg.electronical_shock.push_back(id);
        if (error[OVERLOAD          ]) msg.overload.push_back          (id);
    }
    pub_error_->publish(msg);
}

void DynamixelHandler::BroadcastState_Limit(){
    DynamixelLimit msg;
    // msg.stamp = this->get_clock()->now();
    for (const auto& [id, limit] : limit_r_) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);
        msg.temperature_limit_degc.push_back   (round4(limit[TEMPERATURE_LIMIT ]    ));
        msg.max_voltage_limit_v.push_back      (round4(limit[MAX_VOLTAGE_LIMIT ]    ));
        msg.min_voltage_limit_v.push_back      (round4(limit[MIN_VOLTAGE_LIMIT ]    ));
        msg.pwm_limit_percent.push_back        (round4(limit[PWM_LIMIT         ]    ));
        msg.current_limit_ma.push_back         (round4(limit[CURRENT_LIMIT     ]    ));
        msg.acceleration_limit_deg_ss.push_back(round4(limit[ACCELERATION_LIMIT]/DEG));
        msg.velocity_limit_deg_s.push_back     (round4(limit[VELOCITY_LIMIT    ]/DEG));
        msg.max_position_limit_deg.push_back   (round4(limit[MAX_POSITION_LIMIT]/DEG));
        msg.min_position_limit_deg.push_back   (round4(limit[MIN_POSITION_LIMIT]/DEG));
    }
    pub_limit_->publish(msg);
}

void DynamixelHandler::BroadcastState_Gain(){
    DynamixelGain msg;
    // msg.stamp = this->get_clock()->now();
    for ( const auto& [id, gain] : gain_r_ ) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);
        msg.velocity_i_gain_pulse.push_back     (gain[VELOCITY_I_GAIN     ]);
        msg.velocity_p_gain_pulse.push_back     (gain[VELOCITY_P_GAIN     ]);
        msg.position_d_gain_pulse.push_back     (gain[POSITION_D_GAIN     ]);
        msg.position_i_gain_pulse.push_back     (gain[POSITION_I_GAIN     ]);
        msg.position_p_gain_pulse.push_back     (gain[POSITION_P_GAIN     ]);
        msg.feedforward_2nd_gain_pulse.push_back(gain[FEEDFORWARD_ACC_GAIN]);
        msg.feedforward_1st_gain_pulse.push_back(gain[FEEDFORWARD_VEL_GAIN]);
    }
    pub_gain_->publish(msg);
}

void DynamixelHandler::BroadcastState_Goal(){
    DynamixelGoal msg;
    // msg.stamp = this->get_clock()->now();
    for ( const auto& [id, goal] : goal_r_ ) if ( is_in(id, id_set_) ) {
        msg.id_list.push_back(id);
        msg.pwm_percent.push_back       (round4(goal[GOAL_PWM     ]));
        msg.current_ma.push_back        (round4(goal[GOAL_CURRENT ]));
        msg.velocity_deg_s.push_back    (round4(goal[GOAL_VELOCITY]/DEG));
        msg.profile_vel_deg_s.push_back (round4(goal[PROFILE_VEL  ]/DEG));
        msg.profile_acc_deg_ss.push_back(round4(goal[PROFILE_ACC  ]/DEG));
        msg.position_deg.push_back      (round4(goal[GOAL_POSITION]/DEG));
    }
    pub_goal_->publish(msg);
}