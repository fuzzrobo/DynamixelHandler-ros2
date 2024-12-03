#include "dynamixel_handler.hpp"
#include "myUtils/formatting_output.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp" // enum のインクリメントと， is_in 関数の実装

//* Main loop 内で使う全モータへの一括読み書き関数たち

/**
 * @func SyncWriteGoal
 * @brief 指定した範囲のコマンド値を書き込む
 * @param list_write_goal 書き込むコマンドのEnumのリスト
*/
template <> void DynamixelHandler::SyncWriteGoal(set<GoalIndex> list_write_goal, set<uint8_t> id_set){
    SyncWriteGoal<AddrX>(list_write_goal, id_set);
    SyncWriteGoal<AddrP>(list_write_goal, id_set);
}
template <typename Addr> void DynamixelHandler::SyncWriteGoal(set<GoalIndex> list_write_goal, set<uint8_t> id_set){
    if ( list_write_goal.empty() ) return; // 空なら即時return
    //* 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto [start,end] = minmax_element(list_write_goal.begin(), list_write_goal.end()); 
    if ( use_split_write_ ) end = start; // 分割書き込みが有効な場合は書き込む範囲を1つ目のみに制限
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> goal_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_goal_vec_map; // id と 書き込むデータのベクタのマップ
    for (GoalIndex goal = *start; goal <= *end; goal++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch (goal) {
            case GOAL_PWM      : goal_addr_list.push_back(Addr::goal_pwm            ); break;
            case GOAL_CURRENT  : goal_addr_list.push_back(Addr::goal_current        ); break;
            case GOAL_VELOCITY : goal_addr_list.push_back(Addr::goal_velocity       ); break;
            case PROFILE_ACC   : goal_addr_list.push_back(Addr::profile_acceleration); break;
            case PROFILE_VEL   : goal_addr_list.push_back(Addr::profile_velocity    ); break;
            case GOAL_POSITION : goal_addr_list.push_back(Addr::goal_position       ); break;
            default: /*ここに来たらエラ-*/ exit(1);
        }
        const auto& addr = goal_addr_list.back();
        for (auto id : id_set) if ( series_[id]==Addr::series() ) {
            if ( !is_goal_updated_[id] ) continue;
            id_goal_vec_map[id].push_back( addr.val2pulse( goal_w_[id][goal], model_[id] ) );
        }
    }
    if ( id_goal_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //* id_goal_vec_mapの中身を確認
    if ( verbose_["w_goal"] ) {
        char header[100]; sprintf(header, "'%zu' servo(s) will be written", id_goal_vec_map.size());
        auto ss = control_table_layout(width_log_, id_goal_vec_map, goal_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    //*SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(goal_addr_list, id_goal_vec_map);
    //*再帰的に処理, use_split_write_=falseの場合は全て削除されるので,再帰しない
    list_write_goal.erase(start, ++end);  // 今回書き込んだ範囲を消去
    SyncWriteGoal<Addr>(list_write_goal);
    //*後処理，再帰の終端で実行される, 書き込んだ id のフラグをリセット
    for (const auto& [id, _] : id_goal_vec_map) is_goal_updated_[id] = false;
}

/** 
 * @func SyncWriteGain
 */
template <> void DynamixelHandler::SyncWriteGain(set<GainIndex> list_write_gain, set<uint8_t> id_set){
    SyncWriteGain<AddrX>(list_write_gain, id_set);
    SyncWriteGain<AddrP>(list_write_gain, id_set);
}
template <typename Addr> void DynamixelHandler::SyncWriteGain(set<GainIndex> list_write_gain, set<uint8_t> id_set){
    if ( list_write_gain.empty() ) return; // 空なら即時return
    //* 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto [start,end] = minmax_element(list_write_gain.begin(), list_write_gain.end());
    if ( use_split_write_ || id_set.size()>5  ) end = start; // 分割書き込みを常に有効にする．
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> gain_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_gain_vec_map; // id と 書き込むデータのベクタのマップ
    for (GainIndex gain = *start; gain <= *end; gain++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch (gain) {
            case VELOCITY_I_GAIN     : gain_addr_list.push_back(Addr::velocity_i_gain     ); break;
            case VELOCITY_P_GAIN     : gain_addr_list.push_back(Addr::velocity_p_gain     ); break;
            case POSITION_D_GAIN     : gain_addr_list.push_back(Addr::position_d_gain     ); break;
            case POSITION_I_GAIN     : gain_addr_list.push_back(Addr::position_i_gain     ); break;
            case POSITION_P_GAIN     : gain_addr_list.push_back(Addr::position_p_gain     ); break;
            case FEEDFORWARD_ACC_GAIN: gain_addr_list.push_back(Addr::feedforward_2nd_gain); break;
            case FEEDFORWARD_VEL_GAIN: gain_addr_list.push_back(Addr::feedforward_1st_gain); break;
            default: /*ここに来たらエラ-*/ exit(1);
        }
        const auto& addr = gain_addr_list.back();
        for (auto id : id_set) if ( series_[id]==Addr::series() ) {
            if ( !is_gain_updated_[id] ) continue;
            id_gain_vec_map[id].push_back( addr.val2pulse( gain_w_[id][gain], model_[id] ) );
        }
    }
    if ( id_gain_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //* id_gain_vec_mapの中身を確認
    if ( verbose_["w_gain"] ) {
        char header[100]; sprintf(header, "'%zu' servo(s) will be written", id_gain_vec_map.size());
        auto ss = control_table_layout(width_log_, id_gain_vec_map, gain_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    //*SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(gain_addr_list, id_gain_vec_map);
    //*再帰的に処理
    list_write_gain.erase(start, ++end);  // 今回書き込んだ範囲を消去
    SyncWriteGain<Addr>(list_write_gain, id_set);
    //*後処理，再帰の終端で実行される, 書き込んだ id のフラグをリセット
    for (const auto& [id, _] : id_gain_vec_map) is_gain_updated_[id] = false;
}

/**
 * @func SyncWriteLimit
 * @brief 制限値をすべて書き込む, ROMに書き込むのでトルクをOFFにする
 * @param list_write_limit 書き込む制限値のEnumのset
 */
template <> void DynamixelHandler::SyncWriteLimit(set<LimitIndex> list_write_limit, set<uint8_t> id_set){
    map<uint8_t, int64_t> id_torque_map_x, id_torque_map_p;
    for (auto id : id_set) if ( is_limit_updated_[id] ) {
        if ( series_[id]==AddrX::series() ) id_torque_map_x[id] = TORQUE_DISABLE;
        if ( series_[id]==AddrP::series() ) id_torque_map_p[id] = TORQUE_DISABLE;
    }
    if ( !id_torque_map_x.empty() ) dyn_comm_.SyncWrite(AddrX::torque_enable, id_torque_map_x);
    if ( !id_torque_map_p.empty() ) dyn_comm_.SyncWrite(AddrP::torque_enable, id_torque_map_p);
    SyncWriteLimit<AddrX>(list_write_limit, id_set);
    SyncWriteLimit<AddrP>(list_write_limit, id_set);
}
template <typename Addr> void DynamixelHandler::SyncWriteLimit(set<LimitIndex> list_write_limit, set<uint8_t> id_set){
    if ( list_write_limit.empty() ) return; // 空なら即時return
    //* 書き込む範囲のイテレータを取得, 分割書き込みが有効な場合書き込む範囲を1つ目のみに制限,残りは再帰的に処理する．
    auto [start,end] = minmax_element(list_write_limit.begin(), list_write_limit.end()); 
    if ( use_split_write_ || id_set.size()>5  ) end = start; // 分割書き込みを常に有効にする．
    //* 書き込みに必要な変数を用意
    vector<DynamixelAddress> limit_addr_list;  // 書き込むコマンドのアドレスのベクタ
    map<uint8_t, vector<int64_t>> id_limit_vec_map; // id と 書き込むデータのベクタのマップ
    for (LimitIndex limit = *start; limit <= *end; limit++) { // アドレスのベクタと，データのベクタの並びは対応している必要があるので，同一のループで作成する．
        switch (limit) {
            case TEMPERATURE_LIMIT : limit_addr_list.push_back(Addr::temperature_limit ); break;
            case MAX_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::max_voltage_limit ); break;
            case MIN_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::min_voltage_limit ); break;
            case PWM_LIMIT         : limit_addr_list.push_back(Addr::pwm_limit         ); break;
            case CURRENT_LIMIT     : limit_addr_list.push_back(Addr::current_limit     ); break;
            case ACCELERATION_LIMIT: limit_addr_list.push_back(Addr::acceleration_limit); break;
            case VELOCITY_LIMIT    : limit_addr_list.push_back(Addr::velocity_limit    ); break;
            case MAX_POSITION_LIMIT: limit_addr_list.push_back(Addr::max_position_limit); break;
            case MIN_POSITION_LIMIT: limit_addr_list.push_back(Addr::min_position_limit); break;
            default: /*ここに来たらエラ-*/ std::runtime_error("Unknown LimitIndex");
        }
        const auto& addr = limit_addr_list.back();
        for (auto id : id_set) if ( series_[id]==Addr::series() ) {
            if ( !is_limit_updated_[id] ) continue;
            id_limit_vec_map[id].push_back( addr.val2pulse( limit_w_[id][limit], model_[id] ) );
        }
    }
    if ( id_limit_vec_map.empty() ) return; // 書き込むデータがない場合は即時return
    //* id_limit_vec_mapの中身を確認
    if ( verbose_["w_limit"] ) {
        char header[100]; sprintf(header, "'%zu' servo(s) will be written", id_limit_vec_map.size());
        auto ss = control_table_layout(width_log_, id_limit_vec_map, limit_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    //*SyncWriteでまとめて書き込み
    dyn_comm_.SyncWrite(limit_addr_list, id_limit_vec_map);
    //*再帰的に処理, use_split_write_=falseの場合は全て削除されるので,再帰しない
    list_write_limit.erase(start, ++end);  // 今回書き込んだ範囲を消去
    SyncWriteLimit<Addr>(list_write_limit, id_set);
    //*後処理，再帰の終端で実行される, 書き込んだ id のフラグをリセット
    for (const auto& [id, _] : id_limit_vec_map) is_limit_updated_[id] = false;
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

/**
 * @func SyncReadPresent
 * @brief 指定した範囲の状態値を読み込む
 * @param list_read_state 読み込む状態値のEnumのリスト
 * @return 読み取りの成功率, なんで再帰で頑張って実装してるんだろう．．．
*/
template <>double DynamixelHandler::SyncReadPresent(set<PresentIndex> list_read_state, set<uint8_t> id_set ){
    double suc_rate_X = SyncReadPresent<AddrX>(list_read_state, id_set);
    double suc_rate_P = SyncReadPresent<AddrP>(list_read_state, id_set);
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / (num_[SERIES_X]+num_[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::SyncReadPresent(set<PresentIndex> list_read_state, set<uint8_t> id_set ){
    if ( list_read_state.empty() ) return 1.0; // 空なら即時return
    //* 読み込む範囲のstate_addr_listのインデックスを取得
    auto [start, end] = minmax_element(list_read_state.begin(), list_read_state.end());
    if ( use_split_read_ ) end = start; // 分割読み込みが有効な場合は読み込む範囲を1つ目のみに制限
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> state_addr_list;
    for (PresentIndex st=*start; st<=*end; st++) switch (st) {
        case PRESENT_PWM          : state_addr_list.push_back(Addr::present_pwm          ); break; 
        case PRESENT_CURRENT      : state_addr_list.push_back(Addr::present_current      ); break; 
        case PRESENT_VELOCITY     : state_addr_list.push_back(Addr::present_velocity     ); break;    
        case PRESENT_POSITION     : state_addr_list.push_back(Addr::present_position     ); break;   
        case VELOCITY_TRAJECTORY  : state_addr_list.push_back(Addr::velocity_trajectory  ); break;   
        case POSITION_TRAJECTORY  : state_addr_list.push_back(Addr::position_trajectory  ); break;  
        case PRESENT_INPUT_VOLTAGE: state_addr_list.push_back(Addr::present_input_voltage); break; 
        case PRESENT_TEMPERATURE  : state_addr_list.push_back(Addr::present_temperture   ); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }
    vector<uint8_t> target_id_list;
    for (auto id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return
    // SyncReadでまとめて読み込み
    const auto id_st_vec_map = ( use_fast_read_ ) // fast read を使うかどうか． 途中で切り替えるとtimeout後に来るデータによってSyncReadが何度も失敗するので注意
        ? dyn_comm_.SyncRead_fast(state_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (state_addr_list, target_id_list); fflush(stdout);
    const int N_total = target_id_list.size();
    const int N_suc   = id_st_vec_map.size();
    const bool is_timeout_  = dyn_comm_.timeout_last_read();
    const bool is_comm_err_ = dyn_comm_.comm_error_last_read();
    //* 通信エラーの表示
    if ( verbose_["r_present_err"] ) if ( is_timeout_ || is_comm_err_ ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_st_vec_map.find(id) == id_st_vec_map.end() ) failed_id_list.push_back(id);
        ROS_WARN("'%d' servo(s) failed to read %s", N_total - N_suc, is_timeout_ ? " (time out)" : " (some kind packet error)");
        ROS_WARN_STREAM(id_list_layout(failed_id_list));
    }
    //* id_st_vec_mapの中身を確認
    if ( verbose_["r_present"] ) if ( N_suc>0 ) {
        char header[99]; sprintf(header, "'%d' servo(s) are read", N_suc);
        auto ss = control_table_layout(width_log_, id_st_vec_map, state_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    //* present_r_に反映
    const unsigned int num_state_now  = *end-*start+1;
    for ( size_t i = 0; i < num_state_now; i++ ) {
        const auto addr = state_addr_list[i];
        for (const auto& [id, data_int] : id_st_vec_map)
            present_r_[id][*start+i] = addr.pulse2val( data_int[i], model_[id]);
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, use_split_read_=falseの場合は全て削除されるので,再帰しない
    list_read_state.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int num_state_next = list_read_state.size();
    double suc_rate = SyncReadPresent<Addr>(list_read_state, id_set);
    return       suc_rate       *num_state_next / (num_state_now+num_state_next) // 再帰的な意味で前回までの成功率に重しをかけたもの
         + N_suc/(double)N_total*num_state_now  / (num_state_now+num_state_next);// 今回の成功率に今回読み込んだデータの数を重しとしてかけたもの
}

/**
 * @func SyncReadHardwareError
 * @brief ハードウェアエラーを読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadHardwareErrors(set<uint8_t> id_set){
    double suc_rate_X = SyncReadHardwareErrors<AddrX>(id_set);
    double suc_rate_P = SyncReadHardwareErrors<AddrP>(id_set);
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / (num_[SERIES_X]+num_[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::SyncReadHardwareErrors(set<uint8_t> id_set){
    if ( !has_any_hardware_error_ ) { hardware_err_.clear(); return 1.0; } // 事前にエラーが検出できていない場合は省略

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return
    
    auto id_error_map =  ( use_fast_read_ ) 
        ? dyn_comm_.SyncRead_fast(Addr::hardware_error_status, target_id_list)
        : dyn_comm_.SyncRead     (Addr::hardware_error_status, target_id_list); fflush(stdout);

    if ( dyn_comm_.timeout_last_read() ) return 0.0; // 読み込み失敗

    //  hardware_error_に反映
    for (const auto& [id, error] : id_error_map ){
        hardware_err_[id].fill(false);
        if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE     )& 0b1 ) hardware_err_[id][INPUT_VOLTAGE     ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_HALL_SENSOR )& 0b1 ) hardware_err_[id][MOTOR_HALL_SENSOR ] = true;
        if ((error >> HARDWARE_ERROR_OVERHEATING       )& 0b1 ) hardware_err_[id][OVERHEATING       ] = true;
        if ((error >> HARDWARE_ERROR_MOTOR_ENCODER     )& 0b1 ) hardware_err_[id][MOTOR_ENCODER     ] = true;
        if ((error >> HARDWARE_ERROR_ELECTRONICAL_SHOCK)& 0b1 ) hardware_err_[id][ELECTRONICAL_SHOCK] = true;
        if ((error >> HARDWARE_ERROR_OVERLOAD          )& 0b1 ) hardware_err_[id][OVERLOAD          ] = true;
    }

    // コンソールへの表示
    if ( verbose_["r_hwerr"] ) {
        ROS_WARN( "Hardware error are Checked");
        for (auto id : target_id_list) {
            if (hardware_err_[id][INPUT_VOLTAGE     ]) ROS_WARN (" * servo id [%d] has INPUT_VOLTAGE error"     ,id);
            if (hardware_err_[id][MOTOR_HALL_SENSOR ]) ROS_ERROR(" * servo id [%d] has MOTOR_HALL_SENSOR error" ,id);
            if (hardware_err_[id][OVERHEATING       ]) ROS_ERROR(" * servo id [%d] has OVERHEATING error"       ,id);
            if (hardware_err_[id][MOTOR_ENCODER     ]) ROS_ERROR(" * servo id [%d] has MOTOR_ENCODER error"     ,id);
            if (hardware_err_[id][ELECTRONICAL_SHOCK]) ROS_ERROR(" * servo id [%d] has ELECTRONICAL_SHOCK error",id);
            if (hardware_err_[id][OVERLOAD          ]) ROS_ERROR(" * servo id [%d] has OVERLOAD error"          ,id);
        }
    }
    // 0b00000001 << HARDWARE_ERROR_INPUT_VOLTAGE と error が等しい場合のみ，そのエラーをfalseにする
    for ( const auto& [id, error] : id_error_map ) if ((error >> HARDWARE_ERROR_INPUT_VOLTAGE     )& 0b1 ) hardware_err_[id][INPUT_VOLTAGE] = false;
    return id_error_map.size()/double(target_id_list.size());
}

/**
 * @func SyncReadGain
 * @brief ゲインをすべて読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadGain(set<GainIndex> list_read_gain, set<uint8_t> id_set){
    double suc_rate_X = SyncReadGain<AddrX>(list_read_gain, id_set);
    double suc_rate_P = SyncReadGain<AddrP>(list_read_gain, id_set);
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / (num_[SERIES_X]+num_[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::SyncReadGain(set<GainIndex> list_read_gain, set<uint8_t> id_set){
    if ( list_read_gain.empty() ) return 1.0; // 空なら即時return
    //* 読み込む範囲のgain_addr_listのインデックスを取得
    auto [start, end] = minmax_element(list_read_gain.begin(), list_read_gain.end());
    if ( use_split_read_ ) end = start; // 分割読み込みを常に有効にする．
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> gain_addr_list;
    for (GainIndex g=*start; g<=*end; g++) switch ( g ) {
        case VELOCITY_I_GAIN     : gain_addr_list.push_back(Addr::velocity_i_gain     ); break;
        case VELOCITY_P_GAIN     : gain_addr_list.push_back(Addr::velocity_p_gain     ); break;
        case POSITION_D_GAIN     : gain_addr_list.push_back(Addr::position_d_gain     ); break;
        case POSITION_I_GAIN     : gain_addr_list.push_back(Addr::position_i_gain     ); break;
        case POSITION_P_GAIN     : gain_addr_list.push_back(Addr::position_p_gain     ); break;
        case FEEDFORWARD_ACC_GAIN: gain_addr_list.push_back(Addr::feedforward_2nd_gain); break;
        case FEEDFORWARD_VEL_GAIN: gain_addr_list.push_back(Addr::feedforward_1st_gain); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_gain_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(gain_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (gain_addr_list, target_id_list);  fflush(stdout);
    const int N_total = target_id_list.size();
    const int N_suc   = id_gain_vec_map.size();
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( verbose_["r_gain_err"] ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_gain_vec_map.find(id) == id_gain_vec_map.end() ) failed_id_list.push_back(id);
        ROS_WARN("'%d' servo(s) failed to read %s", N_total - N_suc, is_timeout ? " (time out)" : " (some kind packet error)");
        ROS_WARN_STREAM(id_list_layout(failed_id_list));
    }
    // id_gain_vec_mapの中身を確認
    if ( verbose_["r_gain"] ) if ( N_suc>0 ) {
        char header[100]; sprintf(header, "'%d' servo(s) are read", N_suc);
        auto ss = control_table_layout(width_log_, id_gain_vec_map, gain_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // gain_r_に反映
    const unsigned int num_gain_now  = *end-*start+1;
    for ( size_t i = 0; i < num_gain_now; i++ ) { 
        DynamixelAddress addr = gain_addr_list[i];
        for (const auto& [id, data_int] : id_gain_vec_map)
            gain_r_[id][*start+i] = static_cast<uint16_t>( addr.pulse2val( data_int[i], model_[id] ) ); // pulse2valはdoubleを返すので，uint16_tにキャスト
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, 
    list_read_gain.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int num_gain_next = list_read_gain.size();
    double suc_rate = SyncReadGain<Addr>(list_read_gain, id_set);
    return      suc_rate       *num_gain_next / (num_gain_now+num_gain_next) // 再帰的な意味で前回までの成功率に重しをかけたもの
        + N_suc/(double)N_total*num_gain_now  / (num_gain_now+num_gain_next);// 今回の成功率に今回読み込んだデータの数を重しとしてかけたもの
}

/**
 * @func SyncReadLimit
 * @brief 制限値をすべて読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadLimit(set<LimitIndex> list_read_limit, set<uint8_t> id_set){
    double suc_rate_X = SyncReadLimit<AddrX>(list_read_limit, id_set);
    double suc_rate_P = SyncReadLimit<AddrP>(list_read_limit, id_set);
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / (num_[SERIES_X]+num_[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::SyncReadLimit(set<LimitIndex> list_read_limit, set<uint8_t> id_set){
    if ( list_read_limit.empty() ) return 1.0; // 空なら即時return
    //* 読み込む範囲のlimit_addr_listのインデックスを取得
    auto [start, end] = minmax_element(list_read_limit.begin(), list_read_limit.end());
    if ( use_split_read_ || id_set.size()>5  ) end = start; // 分割読み込みを常に有効にする．
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> limit_addr_list;
    for (LimitIndex l=*start; l<=*end; l++) switch ( l ) {
        case TEMPERATURE_LIMIT : limit_addr_list.push_back(Addr::temperature_limit ); break;
        case MAX_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::max_voltage_limit ); break;
        case MIN_VOLTAGE_LIMIT : limit_addr_list.push_back(Addr::min_voltage_limit ); break;
        case PWM_LIMIT         : limit_addr_list.push_back(Addr::pwm_limit         ); break;
        case CURRENT_LIMIT     : limit_addr_list.push_back(Addr::current_limit     ); break;
        case ACCELERATION_LIMIT: limit_addr_list.push_back(Addr::acceleration_limit); break;
        case VELOCITY_LIMIT    : limit_addr_list.push_back(Addr::velocity_limit    ); break;
        case MAX_POSITION_LIMIT: limit_addr_list.push_back(Addr::max_position_limit); break;
        case MIN_POSITION_LIMIT: limit_addr_list.push_back(Addr::min_position_limit); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_limit_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(limit_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (limit_addr_list, target_id_list); fflush(stdout);
    const int N_total = target_id_list.size();
    const int N_suc   = id_limit_vec_map.size();
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( verbose_["r_limit_err"] ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_limit_vec_map.find(id) == id_limit_vec_map.end() ) failed_id_list.push_back(id);
        ROS_WARN("'%d' servo(s) failed to read %s", N_total - N_suc, is_timeout ? " (time out)" : " (some kind packet error)");
        ROS_WARN_STREAM(id_list_layout(failed_id_list));
    }
    // ACCELERATION_LIMITに関してだけ修正を入れる．0はほぼあり得ないかつ0の時profile_accの設定ができないので，適当に大きな値に変更する．
    vector<uint8_t> fixed_id_list;
    if (*start <= ACCELERATION_LIMIT && ACCELERATION_LIMIT <= *end) 
        for (auto& [id, limit] : id_limit_vec_map) {
            if ( limit[ACCELERATION_LIMIT-*start] != 0 ) continue;
            fixed_id_list.push_back(id);
            limit[ACCELERATION_LIMIT-*start] = 32767; //  Xシリーズのprofile_accの設定ができる最大値
        }
    // id_limit_vec_mapの中身を確認
    if ( verbose_["r_limit"] ) if ( N_suc>0 ) {
        char header[100]; sprintf(header, "'%d' servo(s) are read", N_suc);
        auto ss = control_table_layout(width_log_, id_limit_vec_map, limit_addr_list, string(header));
        ROS_INFO_STREAM(ss);
        if ( !fixed_id_list.empty() ) {
            char header[100]; sprintf(header,"\n[%d] servo(s)' accelerarion_limit is 0, change to 32767", (int)fixed_id_list.size());
            auto ss = id_list_layout(fixed_id_list, string(header));
            ROS_WARN_STREAM(ss);
        }
    }
    // limit_r_に反映
    const unsigned int num_limit_now  = *end-*start+1;
    for ( size_t i = 0; i < num_limit_now; i++ ) {
        DynamixelAddress addr = limit_addr_list[i];
        for (const auto& [id, data_int] : id_limit_vec_map)
            limit_r_[id][*start+i] = addr.pulse2val( data_int[i], model_[id] );
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理,
    list_read_limit.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int num_limit_next = list_read_limit.size();
    double suc_rate = SyncReadLimit<Addr>(list_read_limit, id_set);
    return    suc_rate         *num_limit_next / (num_limit_now+num_limit_next) // 再帰的な意味で前回までの成功率に重しをかけたもの
        + N_suc/(double)N_total*num_limit_now  / (num_limit_now+num_limit_next);// 今回の成功率に今回読み込んだデータの数を重しとしてかけたもの
}

/**
 * @func SyncReadGoal
 * @brief 目標値をすべて読み込む
 * @return 読み取りの成功率
*/
template <> double DynamixelHandler::SyncReadGoal(set<GoalIndex> list_read_goal, set<uint8_t> id_set){
    double suc_rate_X = SyncReadGoal<AddrX>(list_read_goal, id_set);
    double suc_rate_P = SyncReadGoal<AddrP>(list_read_goal, id_set);
    return (suc_rate_X * num_[SERIES_X] + suc_rate_P * num_[SERIES_P]) / (num_[SERIES_X]+num_[SERIES_P]);
}
template <typename Addr> double DynamixelHandler::SyncReadGoal(set<GoalIndex> list_read_goal, set<uint8_t> id_set){
    if ( list_read_goal.empty() ) return 1.0; // 空なら即時return
    //* 読み込む範囲のgoal_addr_listのインデックスを取得
    auto [start, end] = minmax_element(list_read_goal.begin(), list_read_goal.end());
    if ( use_split_read_ || id_set.size()>5 ) end = start; // 分割読み込みを常に有効にする．
    //* 読み込みに必要な変数を用意
    vector<DynamixelAddress> goal_addr_list;
    for (GoalIndex g=*start; g<=*end; g++) switch ( g ) {
        case GOAL_PWM      : goal_addr_list.push_back(Addr::goal_pwm      ); break;
        case GOAL_CURRENT  : goal_addr_list.push_back(Addr::goal_current  ); break;
        case GOAL_VELOCITY : goal_addr_list.push_back(Addr::goal_velocity ); break;
        case PROFILE_ACC   : goal_addr_list.push_back(Addr::profile_acceleration); break;
        case PROFILE_VEL   : goal_addr_list.push_back(Addr::profile_velocity    ); break;
        case GOAL_POSITION : goal_addr_list.push_back(Addr::goal_position       ); break;
        default: /*ここに来たらエラ-*/ exit(1);
    }

    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return 1.0; // 読み込むデータがない場合は即時return

    auto id_goal_vec_map = ( use_fast_read_ )
        ? dyn_comm_.SyncRead_fast(goal_addr_list, target_id_list)
        : dyn_comm_.SyncRead     (goal_addr_list, target_id_list);  fflush(stdout);
    const int N_total = target_id_list.size();
    const int N_suc   = id_goal_vec_map.size();
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    // 通信エラーの表示
    if ( verbose_["r_goal_err"] ) if ( has_comm_err || is_timeout ) {
        vector<uint8_t> failed_id_list;
        for ( auto id : target_id_list ) if ( id_goal_vec_map.find(id) == id_goal_vec_map.end() ) failed_id_list.push_back(id);
        ROS_WARN("'%d' servo(s) failed to read %s", N_total - N_suc, is_timeout ? " (time out)" : " (some kind packet error)");
        ROS_WARN_STREAM(id_list_layout(failed_id_list));
    }
    if ( verbose_["r_goal"] ) if ( N_suc>0 ) {
        char header[100]; sprintf(header, "'%d' servo(s) are read", N_suc);
        auto ss = control_table_layout(width_log_, id_goal_vec_map, goal_addr_list, string(header));
        ROS_INFO_STREAM(ss);
    }
    // goal_r_に反映
    const unsigned int  num_goal_now  = *end-*start+1;
    for ( size_t i = 0; i < num_goal_now; i++) {
        DynamixelAddress addr = goal_addr_list[i];
        for (const auto& [id, data_int] : id_goal_vec_map)
            goal_r_[id][*start+i] = addr.pulse2val( data_int[i], model_[id] );
    }
    // 今回読み込んだ範囲を消去して残りを再帰的に処理, 
    list_read_goal.erase(start, ++end); // 今回読み込んだ範囲を消去
    const unsigned int  num_goal_next = list_read_goal.size();
    double suc_rate = SyncReadGoal<Addr>(list_read_goal, id_set);
    return    suc_rate         *num_goal_next / (num_goal_now+num_goal_next) // 再帰的な意味で前回までの成功率に重しをかけたもの
        + N_suc/(double)N_total*num_goal_now  / (num_goal_now+num_goal_next);// 今回の成功率に今回読み込んだデータの数を重しとしてかけたもの
}

// 全てのモータの動作を停止させる．
template <> void DynamixelHandler::StopDynamixels(set<uint8_t> id_set){
    StopDynamixels<AddrX>(id_set);
    StopDynamixels<AddrP>(id_set);
} 
template <typename Addr> void DynamixelHandler::StopDynamixels(set<uint8_t> id_set){
    vector<uint8_t> id_list; 
    for (auto id : id_set) if ( series_[id]==Addr::series() ) id_list.push_back(id);
    auto offset_pulse_now = dyn_comm_.SyncRead(Addr::homing_offset, id_list);
    vector<int64_t> offset_pulse(id_list.size(), 0);
    dyn_comm_.SyncWrite(Addr::homing_offset, id_list, offset_pulse); // マジで謎だが，BusWatchdogを設定するとHomingOffset分だけ回転してしまう...多分ファームrウェアのバグ
    vector<int64_t> bus_watchtime_pulse(id_list.size(), 1);
    dyn_comm_.SyncWrite(Addr::bus_watchdog, id_list, bus_watchtime_pulse);
    dyn_comm_.SyncWrite(Addr::homing_offset, offset_pulse_now);
    ROS_INFO("%s servo will be stopped", Addr::series()==SERIES_X ? "X series" 
                                       : Addr::series()==SERIES_P ? "P series" : "Unknown");
}

template <> void DynamixelHandler::CheckDynamixels(set<uint8_t> id_set){
    has_hardware_error_.clear();
    CheckDynamixels<AddrX>(id_set);
    has_any_hardware_error_ = dyn_comm_.hardware_error_last_read();
    CheckDynamixels<AddrP>(id_set);
    has_any_hardware_error_ |= dyn_comm_.hardware_error_last_read();
    if ( has_any_hardware_error_ ) ROS_WARN( "Hardware Error are detected");
}
template <typename Addr> void DynamixelHandler::CheckDynamixels(set<uint8_t> id_set){
    vector<uint8_t> target_id_list;
    for (int id : id_set) if ( series_[id]==Addr::series() ) target_id_list.push_back(id);
    if ( target_id_list.empty() ) return; // 読み込むデータがない場合は即時return

    // トルクの確認
    auto id_torque_map = dyn_comm_.SyncRead( Addr::torque_enable, target_id_list );
    for ( const auto& [id, torque] : id_torque_map ) tq_mode_[id] = torque;

    // ハードウェアエラーの確認
    auto error_id_list = dyn_comm_.hardware_error_id_last_read();
    for (auto id : error_id_list) has_hardware_error_[id] = true;

    // 直前に通信エラーがあれば，モータの応答を確認
    const bool is_timeout   = dyn_comm_.timeout_last_read();
    const bool has_comm_err = dyn_comm_.comm_error_last_read();
    if ( !is_timeout && !has_comm_err ) { ping_err_.clear(); return; } // 通信エラーがない場合は即時return

    vector<uint8_t> alive_id_list;
    for (auto id : target_id_list) if ( dyn_comm_.Ping(id) ) alive_id_list.push_back(id);
    // すべてのモータが死んでいる場合は，ping_err_=1で固定とする, おそらく根本で電源が切断されているため．
    // 生き残ったモータのping_err_をリセット
    // 一部の生き残っていないモータのping_err_をインクリメント
    for (auto id : target_id_list) { // 効率が悪いが, わかりやすさを優先
        if ( alive_id_list.empty() )    { ping_err_[id] = 1; continue; }
        if ( is_in(id, alive_id_list) ) { ping_err_[id] = 0; continue; }
        /*  !is_in(id, alive_id_list)  */ ping_err_[id]++;
        ROS_WARN("Servo id [%d] is dead (%d count / %s)", id, (int)ping_err_[id],
            auto_remove_count_ ? (std::to_string(auto_remove_count_)+"count").c_str() : "inf");
    }
}