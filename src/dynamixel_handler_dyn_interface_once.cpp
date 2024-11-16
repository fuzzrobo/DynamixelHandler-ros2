#include "dynamixel_handler.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp"
// 一定時間待つための関数
static void rsleep(int millisec) { std::this_thread::sleep_for(std::chrono::milliseconds(millisec));}
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる

//* 基本機能をまとめた関数たち

// 各シリーズのDynamixelを検出する．
uint8_t DynamixelHandler::ScanDynamixels(uint8_t id_min, uint8_t id_max, uint32_t num_expected, uint32_t times_retry) {
    id_set_.clear();
    ROS_INFO("Scanning: ");
    for (int id = id_min; id <= id_max; id++){
        addDynamixel(id);
        if ( !is_in(id, id_set_) && is_in(id-1, id_set_) ) ROS_INFO("Scanning: ");
        if ( !is_in(id, id_set_)                         ) ROS_INFO("          %c[1A%d", 0x1b, id);
        if ( !rclcpp::ok() ) return 0;
    } 
    auto num_found = id_set_.size();
    // 再帰から脱する条件
    if ( times_retry <= 0 ) return num_found;
    if ( num_found != 0 && num_found >= num_expected ) return num_found;
    // 再帰処理
    if ( num_found < num_expected )  
        ROS_WARN( "Less expected number of Dynamixel are found,\n > %d times retry left", times_retry );
    if ( num_expected == 0 )
        ROS_WARN( "Dynamixels are not found yet,\n > %d times retry left", times_retry );
    rsleep(100);
    return ScanDynamixels(id_min, id_max, num_expected, times_retry-1);
}

bool DynamixelHandler::addDynamixel(uint8_t id){
    if ( is_in(id, id_set_) ) return true;
    if ( !dyn_comm_.tryPing(id) ) return false;

    auto dyn_model = dyn_comm_.tryRead(AddrCommon::model_number, id);
    switch ( dynamixel_series(dyn_model) ) { 
        case SERIES_X: ROS_INFO(" * X series servo id [%d] is found", id);
            model_[id] = dyn_model;
            series_[id] = SERIES_X;
            id_set_.insert(id);
            num_[SERIES_X]++; break;
        case SERIES_P: ROS_INFO(" * P series servo id [%d] is found", id);
            model_[id] = dyn_model;
            series_[id] = SERIES_P;
            id_set_.insert(id);
            num_[SERIES_P]++; break;
        default: ROS_WARN(" * Unkwon model [%d] servo id [%d] is found", (int)dyn_model, id);
            return false;
    }

    WriteBusWatchdog (id, 0.0 );
    WriteHomingOffset(id, 0.0 );
    WriteProfileAcc(id, default_profile_acc_deg_ss_*DEG ); 
    WriteProfileVel(id, default_profile_vel_deg_s_*DEG );

    set<uint8_t> tmp = {id};
    while ( rclcpp::ok() && SyncReadPresent( list_read_present_, tmp) < 1.0-1e-6 ) rsleep(50);
    while ( rclcpp::ok() && SyncReadGoal   ( list_read_goal_ , tmp) < 1.0-1e-6 ) rsleep(50); 
    while ( rclcpp::ok() && SyncReadGain   ( list_read_gain_ , tmp) < 1.0-1e-6 ) rsleep(50); 
    while ( rclcpp::ok() && SyncReadLimit  ( list_read_limit_, tmp) < 1.0-1e-6 ) rsleep(50); 
    while ( rclcpp::ok() && SyncReadHardwareErrors(tmp) < 1.0-1e-6 ) rsleep(50);

    tq_mode_[id] = ReadTorqueEnable(id);
    op_mode_[id] = ReadOperatingMode(id);
    dv_mode_[id] = ReadDriveMode(id);
    limit_w_[id] = limit_r_[id];
    gain_w_[id] = gain_r_[id];
    goal_w_[id] = goal_r_[id];

    if ( abs(default_profile_acc_deg_ss_*DEG - goal_r_[id][PROFILE_ACC]) > 0.1 ) 
        ROS_WARN("\nProfile acceleration is not set correctly [%d],\n your setting [%f], but [%f]", id, default_profile_acc_deg_ss_*DEG, goal_r_[id][PROFILE_ACC]);
    if ( abs(default_profile_vel_deg_s_*DEG - goal_r_[id][PROFILE_VEL]) > 0.1 ) 
        ROS_WARN("\nProfile velocity is not set correctly [%d],\n your setting [%f], but [%f]", id, default_profile_vel_deg_s_*DEG, goal_r_[id][PROFILE_VEL]);

    if ( do_clean_hwerr_ ) ClearHardwareError(id); // 現在の状態を変えない
    if ( do_torque_on_ )   TorqueOn(id);           // 現在の状態を変えない
    return true;
}

bool DynamixelHandler::RemoveDynamixel(uint8_t id){
    if ( !is_in(id, id_set_) ) return true;
    id_set_.erase(id);
    num_[series_[id]]--;
    ROS_INFO("ID [%d] is removed", id);
    return true;
}

// 回転数が消えることを考慮して，モータをリブートする．
bool DynamixelHandler::ClearHardwareError(uint8_t id){
    if ( !is_in(id, id_set_) ) return false;
    if ( !has_hardware_error_[id] ) return true; // エラーがない場合は何もしない

    const auto now_pos = ReadPresentPosition(id); // 失敗すると0が返って危ないので成功した場合だけリブート処理を行う
    const bool pos_success = !dyn_comm_.timeout_last_read() && !dyn_comm_.comm_error_last_read();
    const auto now_offset = ReadHomingOffset(id); // 失敗すると0が返って危ないので成功した場合だけリブート処理を行う
    const bool offset_success = !dyn_comm_.timeout_last_read() && !dyn_comm_.comm_error_last_read();
    if ( pos_success && offset_success ) {
        int now_rot = (now_pos-now_offset+M_PI) / (2*M_PI);
        if (now_pos < -M_PI) now_rot--;
        const double offset = now_offset+now_rot*(2*M_PI);
        /*リブート処理*/dyn_comm_.Reboot(id); //** RAMのデータが消えるが，この処理の後は電源喪失と同じ扱いなので，ここでは気にしない．
        // homing offsetが書き込めるまで待機する．
        while ( !WriteHomingOffset(id, offset) && rclcpp::ok() ) rsleep(10);
    }
    // 結果を確認
    bool is_clear = (ReadHardwareError(id) == 0b00000000);
    if (is_clear) ROS_INFO ("ID [%d] is cleared error", id);
    else          ROS_ERROR("ID [%d] failed to clear error", id);
    return is_clear;
}

// モータの動作モードを変更する．連続で変更するときは1秒のインターバルを入れる
bool DynamixelHandler::ChangeOperatingMode(uint8_t id, DynamixelOperatingMode mode){
    if ( !is_in(id, id_set_) ) return false;
    if ( op_mode_[id] == mode ) return true; // 既に同じモードの場合は何もしない
    if ( get_clock()->now().seconds() - when_op_mode_updated_[id] < 1.0 ) rsleep(1000); // 1秒以内に変更した場合は1秒待つ
    // 変更前のトルク状態を確認
    const bool prev_torque = ReadTorqueEnable(id); // read失敗しても0が返ってくるので問題ない
    WriteTorqueEnable(id, false);
    /*モード変更*/WriteOperatingMode(id, mode);  //**RAMのデータが消えるので注意, これは電源喪失とは異なるのでRAMデータの回復を入れる
    // goal_w_を全部書き込んで，本体とこのプログラムの同期行う．
    if ( op_mode_[id] != OPERATING_MODE_PWM      )  WriteGoalPWM     (id, goal_w_[id][GOAL_PWM     ]);
    if ( op_mode_[id] != OPERATING_MODE_CURRENT  )  WriteGoalCurrent (id, goal_w_[id][GOAL_CURRENT ]);
    if ( op_mode_[id] != OPERATING_MODE_VELOCITY )  WriteGoalVelocity(id, goal_w_[id][GOAL_VELOCITY]);
    WriteProfileAcc  (id, goal_w_[id][PROFILE_ACC  ]);
    WriteProfileVel  (id, goal_w_[id][PROFILE_VEL  ]);
    WriteGoalPosition(id, goal_w_[id][GOAL_POSITION]);
    // WriteGains(id, gain_r_[id]);　// ** Gain値のデフォルトも変わる．面倒な．．．
    double bus_watchdog = mode==OPERATING_MODE_CURRENT  ? 2500 :
                          mode==OPERATING_MODE_VELOCITY ? 2500 : 0;
    WriteBusWatchdog(id, bus_watchdog); // Dynamixel側のバグで書き込んでもうまくいかないことがある．
    WriteTorqueEnable(id, prev_torque == TORQUE_ENABLE );
    // 結果を確認
    bool is_changed = (ReadOperatingMode(id) == mode);
    if ( is_changed ) {
        op_mode_[id] = mode;
        when_op_mode_updated_[id] = get_clock()->now().seconds();
        ROS_INFO("ID [%d] is changed operating mode [%d]", id, mode);
    } else {
        ROS_ERROR("ID [%d] failed to change operating mode", id); 
    }
    return is_changed;
}

// モータを停止させてからトルクを入れる．
bool DynamixelHandler::TorqueOn(uint8_t id){
    if ( !is_in(id, id_set_) ) return false;
    if ( tq_mode_[id] == TORQUE_ENABLE ) return true; // 既にトルクが入っている場合は何もしない
    // dynamixel内のgoal値とこのプログラム内のgoal_w_を一致させる．
    const auto now_pos = ReadPresentPosition(id); // 失敗すると0が返って危ないので確認する
    if ( !( dyn_comm_.timeout_last_read() || dyn_comm_.comm_error_last_read() )){
        // 急に動き出さないように，以下のgoal_w_を設定する
        goal_w_[id][GOAL_POSITION] = now_pos; // トルクがオフならDynamixel本体のgoal_positionはpresent_positionと一致している．
        if (op_mode_[id]==OPERATING_MODE_VELOCITY) goal_w_[id][GOAL_VELOCITY] = 0.0;
        if (op_mode_[id]==OPERATING_MODE_CURRENT ) goal_w_[id][GOAL_CURRENT ] = 0.0;
        if (op_mode_[id]==OPERATING_MODE_PWM     ) goal_w_[id][GOAL_PWM     ] = 0.0;
        // goal_w_を全部書き込んで，本体とこのプログラムの同期行う．
        WriteGoalPWM     (id, goal_w_[id][GOAL_PWM     ]);
        WriteGoalCurrent (id, goal_w_[id][GOAL_CURRENT ]);
        WriteGoalVelocity(id, goal_w_[id][GOAL_VELOCITY]);
        WriteProfileAcc  (id, goal_w_[id][PROFILE_ACC  ]);
        WriteProfileVel  (id, goal_w_[id][PROFILE_VEL  ]);
        WriteGoalPosition(id, goal_w_[id][GOAL_POSITION]);
        // WriteGains(id, gain_r_[id]); 　// その他電源喪失時に消えるデータを念のため書き込む
        double bus_watchdog = op_mode_[id]==OPERATING_MODE_CURRENT  ? 2500 :
                              op_mode_[id]==OPERATING_MODE_VELOCITY ? 2500 : 0;
        WriteBusWatchdog(id, bus_watchdog); // Dynamixel側のバグで書き込んでもうまくいかないことがある．
        /*トルクを入れる*/WriteTorqueEnable(id, true);
    }
    // 結果を確認
    tq_mode_[id] = ReadTorqueEnable(id);
    if ( tq_mode_[id] != TORQUE_ENABLE ) ROS_ERROR("ID [%d] failed to enable torque", id);
                                    else ROS_INFO( "ID [%d] is enabled torque"      , id);
    return tq_mode_[id];
}

// トルクを切る
bool DynamixelHandler::TorqueOff(uint8_t id){
    if ( !is_in(id, id_set_) ) return false;
    if ( tq_mode_[id] == TORQUE_DISABLE ) return true; // 既にトルクが切られている場合は何もしない
    // トルクを切る
    WriteTorqueEnable(id, false);
    // 結果を確認
    tq_mode_[id] = ReadTorqueEnable(id);
    if ( tq_mode_[id] != TORQUE_DISABLE ) ROS_ERROR("ID [%d] failed to disable torque", id);
    return tq_mode_[id];
}

//* 基本機能たち Read
uint8_t DynamixelHandler::ReadHardwareError(uint8_t id){
    return series_[id]==SERIES_X ? dyn_comm_.tryRead(AddrX::hardware_error_status, id) 
          :series_[id]==SERIES_P ? dyn_comm_.tryRead(AddrP::hardware_error_status, id) : 0;
}

bool DynamixelHandler::ReadTorqueEnable(uint8_t id){
    return series_[id]==SERIES_X ? dyn_comm_.tryRead(AddrX::torque_enable, id) 
          :series_[id]==SERIES_P ? dyn_comm_.tryRead(AddrP::torque_enable, id) : false;
}

double DynamixelHandler::ReadPresentPWM(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::present_pwm
               :series_[id]==SERIES_P ? AddrP::present_pwm : AddrX::present_pwm;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadPresentCurrent(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::present_current
               :series_[id]==SERIES_P ? AddrP::present_current : AddrX::present_current;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadPresentVelocity(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::present_velocity
               :series_[id]==SERIES_P ? AddrP::present_velocity : AddrX::present_velocity;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadPresentPosition(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::present_position
               :series_[id]==SERIES_P ? AddrP::present_position : AddrX::present_position;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadGoalPWM(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_pwm
               :series_[id]==SERIES_P ? AddrP::goal_pwm : AddrX::goal_pwm;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadGoalCurrent(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_current
               :series_[id]==SERIES_P ? AddrP::goal_current : AddrX::goal_current;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadGoalVelocity(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_velocity
               :series_[id]==SERIES_P ? AddrP::goal_velocity : AddrX::goal_velocity;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadGoalPosition(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_position
               :series_[id]==SERIES_P ? AddrP::goal_position : AddrX::goal_position;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadProfileAcc(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::profile_acceleration
               :series_[id]==SERIES_P ? AddrP::profile_acceleration : AddrX::profile_acceleration;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadProfileVel(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::profile_velocity
               :series_[id]==SERIES_P ? AddrP::profile_velocity : AddrX::profile_velocity;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadHomingOffset(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::homing_offset
               :series_[id]==SERIES_P ? AddrP::homing_offset : AddrX::homing_offset;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

double DynamixelHandler::ReadBusWatchdog(uint8_t id){
    auto addr = series_[id]==SERIES_X ? AddrX::bus_watchdog
               :series_[id]==SERIES_P ? AddrP::bus_watchdog : AddrX::bus_watchdog;
    return addr.pulse2val(dyn_comm_.tryRead(addr, id), model_[id]);
}

uint8_t DynamixelHandler::ReadOperatingMode(uint8_t id){
    return dyn_comm_.tryRead(AddrCommon::operating_mode, id);
}

uint8_t DynamixelHandler::ReadDriveMode(uint8_t id){
    return dyn_comm_.tryRead(AddrCommon::drive_mode, id);
}

//* 基本機能たち Write

bool DynamixelHandler::WriteTorqueEnable(uint8_t id, bool enable){
    auto addr = series_[id]==SERIES_X ? AddrX::torque_enable
               :series_[id]==SERIES_P ? AddrP::torque_enable : AddrX::torque_enable;
    return dyn_comm_.tryWrite(addr, id, enable ? TORQUE_ENABLE : TORQUE_DISABLE);
}

bool DynamixelHandler::WriteGoalPosition(uint8_t id, double pos){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_position
               :series_[id]==SERIES_P ? AddrP::goal_position : AddrX::goal_position;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(pos, model_[id]));
}

bool DynamixelHandler::WriteGoalPWM(uint8_t id, double pwm){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_pwm
               :series_[id]==SERIES_P ? AddrP::goal_pwm : AddrX::goal_pwm;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(pwm, model_[id]));
}

bool DynamixelHandler::WriteGoalCurrent(uint8_t id, double cur){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_current
               :series_[id]==SERIES_P ? AddrP::goal_current : AddrX::goal_current;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(cur, model_[id]));
}

bool DynamixelHandler::WriteGoalVelocity(uint8_t id, double vel){
    auto addr = series_[id]==SERIES_X ? AddrX::goal_velocity
               :series_[id]==SERIES_P ? AddrP::goal_velocity : AddrX::goal_velocity;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(vel, model_[id]));
}

bool DynamixelHandler::WriteProfileAcc(uint8_t id, double acc){
    auto addr = series_[id]==SERIES_X ? AddrX::profile_acceleration
               :series_[id]==SERIES_P ? AddrP::profile_acceleration : AddrX::profile_acceleration;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(acc, model_[id]));
}

bool DynamixelHandler::WriteProfileVel(uint8_t id, double vel){
    auto addr = series_[id]==SERIES_X ? AddrX::profile_velocity
               :series_[id]==SERIES_P ? AddrP::profile_velocity : AddrX::profile_velocity;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(vel, model_[id]));
}

bool DynamixelHandler::WriteHomingOffset(uint8_t id, double offset){
    auto addr = series_[id]==SERIES_X ? AddrX::homing_offset
               :series_[id]==SERIES_P ? AddrP::homing_offset : AddrX::homing_offset;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(offset, model_[id]));
}
bool DynamixelHandler::WriteBusWatchdog(uint8_t id, double time){
    auto addr = series_[id]==SERIES_X ? AddrX::bus_watchdog
               :series_[id]==SERIES_P ? AddrP::bus_watchdog : AddrX::bus_watchdog;
    return dyn_comm_.tryWrite(addr, id, addr.val2pulse(time, model_[id]));
}

bool DynamixelHandler::WriteGains(uint8_t id, array<uint16_t, _num_gain> gains){
    bool is_success = true;
    if ( series_[id] != SERIES_X ) {
        is_success &= dyn_comm_.tryWrite(AddrX::velocity_i_gain, id, gains[VELOCITY_I_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrX::velocity_p_gain, id, gains[VELOCITY_P_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrX::position_d_gain, id, gains[POSITION_D_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrX::position_i_gain, id, gains[POSITION_I_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrX::position_p_gain, id, gains[POSITION_P_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrX::feedforward_2nd_gain, id, gains[FEEDFORWARD_ACC_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrX::feedforward_1st_gain, id, gains[FEEDFORWARD_VEL_GAIN]);
    } else if ( series_[id] != SERIES_P ) {
        is_success &= dyn_comm_.tryWrite(AddrP::velocity_i_gain, id, gains[VELOCITY_I_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrP::velocity_p_gain, id, gains[VELOCITY_P_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrP::position_d_gain, id, gains[POSITION_D_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrP::position_i_gain, id, gains[POSITION_I_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrP::position_p_gain, id, gains[POSITION_P_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrP::feedforward_2nd_gain, id, gains[FEEDFORWARD_ACC_GAIN]);
        is_success &= dyn_comm_.tryWrite(AddrP::feedforward_1st_gain, id, gains[FEEDFORWARD_VEL_GAIN]);
    } else { is_success = false; }
    return is_success;
}

bool DynamixelHandler::WriteOperatingMode(uint8_t id, uint8_t mode){ 
    return dyn_comm_.tryWrite(AddrCommon::operating_mode, id, mode);
}
