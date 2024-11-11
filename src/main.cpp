#include "dynamixel_handler.hpp"
#include "rclcpp/rclcpp.hpp"
#include "myUtils/logging_like_ros1.hpp"
#include "myUtils/make_iterator_convenient.hpp"
static constexpr double DEG = M_PI/180.0; // degを単位に持つ数字に掛けるとradになる

// 一定時間待つための関数
static void rsleep(int millisec) { std::this_thread::sleep_for(std::chrono::milliseconds(millisec));}

using std::bind;
using std::placeholders::_1;

DynamixelHandler::DynamixelHandler() : Node("dynamixel_handler", rclcpp::NodeOptions()
                                                                  .allow_undeclared_parameters(true)
                                                                  .automatically_declare_parameters_from_overrides(true)) {
    ROS_INFO( "Initializing DynamixelHandler .....");

    bool is_debug; get_parameter_or("debug", is_debug, false);

    // 通信の開始
    int baudrate      ; this->get_parameter_or("baudrate"     , baudrate     , 57600                 );
    int latency_timer ; this->get_parameter_or("latency_timer", latency_timer,    16                 );
    string device_name; this->get_parameter_or("device_name"  , device_name  , string("/dev/ttyUSB0"));

    dyn_comm_ = DynamixelCommunicator(device_name.c_str(), baudrate, latency_timer);
    if ( !dyn_comm_.OpenPort() ) { fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する
        ROS_ERROR("Failed to open USB device [%s]", dyn_comm_.port_name().c_str()); 
        if ( !is_debug ) throw std::runtime_error("Initialization failed (device open)");
    } 
    // serial通信のverbose設定
    bool serial_verbose; get_parameter_or("dyn_comm/verbose", serial_verbose, false);
    dyn_comm_.set_verbose(serial_verbose); fflush(stdout);

    // serial通信のretry設定
    int num_try      ; get_parameter_or("dyn_comm/retry_num"   , num_try      ,  5);
    int msec_interval; get_parameter_or("dyn_comm/inerval_msec", msec_interval, 10);
    dyn_comm_.set_retry_config(num_try, msec_interval); fflush(stdout);

    // main loop の設定
    this->get_parameter_or("loop_rate", loop_rate_, 50u);
    this->get_parameter_or("verbose_ratio", ratio_mainloop_ , 100u);
    this->get_parameter_or("pub_ratio/present.pwm"                 , pub_ratio_present_[PRESENT_PWM          ],  0u);
    this->get_parameter_or("pub_ratio/present.current"             , pub_ratio_present_[PRESENT_CURRENT      ],  1u);
    this->get_parameter_or("pub_ratio/present.velocity"            , pub_ratio_present_[PRESENT_VELOCITY     ],  1u);
    this->get_parameter_or("pub_ratio/present.position"            , pub_ratio_present_[PRESENT_POSITION     ],  1u);
    this->get_parameter_or("pub_ratio/present.velocity_trajectory" , pub_ratio_present_[VELOCITY_TRAJECTORY  ],  0u);
    this->get_parameter_or("pub_ratio/present.position_trajectory" , pub_ratio_present_[POSITION_TRAJECTORY  ],  0u);
    this->get_parameter_or("pub_ratio/present.input_voltage"       , pub_ratio_present_[PRESENT_INPUT_VOLTAGE], 10u);
    this->get_parameter_or("pub_ratio/present.temperature"         , pub_ratio_present_[PRESENT_TEMPERATURE   ], 10u);
    this->get_parameter_or("pub_ratio/status" , pub_ratio_["status"], 50u);
    this->get_parameter_or("pub_ratio/goal"   , pub_ratio_["goal"] ,   0u);
    this->get_parameter_or("pub_ratio/gain"   , pub_ratio_["gain"] ,   0u);
    this->get_parameter_or("pub_ratio/limit"  , pub_ratio_["limit"],   0u);
    this->get_parameter_or("pub_ratio/error"  , pub_ratio_["error"], 100u);
    this->get_parameter_or("max_log_width"     , width_log_      ,   7u);
    this->get_parameter_or("use/split_write"    , use_split_write_    , false);
    this->get_parameter_or("use/split_read"     , use_split_read_     , false);
    this->get_parameter_or("use/fast_read"      , use_fast_read_      , true);
    this->get_parameter_or("verbose/callback"           , verbose_callback_, false);
    this->get_parameter_or("verbose/write_goal"         , verbose_["w_goal"  ], false);
    this->get_parameter_or("verbose/write_gain"         , verbose_["w_gain"  ], false);
    this->get_parameter_or("verbose/write_limit"        , verbose_["w_limit" ], false);
    this->get_parameter_or("verbose/read_status/raw"    , verbose_["r_status"    ], false);
    this->get_parameter_or("verbose/read_status/err"    , verbose_["r_status_err"], false);
    this->get_parameter_or("verbose/read_present/raw"   , verbose_["r_present"    ], false);
    this->get_parameter_or("verbose/read_present/err"   , verbose_["r_present_err"], false);
    this->get_parameter_or("verbose/read_goal/raw"      , verbose_["r_goal"    ], false);
    this->get_parameter_or("verbose/read_goal/err"      , verbose_["r_goal_err"], false);
    this->get_parameter_or("verbose/read_gain/raw"      , verbose_["r_gain"    ], false);
    this->get_parameter_or("verbose/read_gain/err"      , verbose_["r_gain_err"], false);
    this->get_parameter_or("verbose/read_limit/raw"     , verbose_["r_limit"    ], false);
    this->get_parameter_or("verbose/read_limit/err"     , verbose_["r_limit_err"], false);
    this->get_parameter_or("verbose/read_hardware_error", verbose_["r_hwerr" ], false);
    this->get_parameter_or("middle/no_response_id_auto_remove_count", auto_remove_count_   , 0u);

    // id_listの作成
    int num_expected; this->get_parameter_or("init/expected_servo_num"     , num_expected, 0);
    int times_retry ; this->get_parameter_or("init/auto_search_retry_times", times_retry , 5);
    int id_min      ; this->get_parameter_or("init/auto_search_min_id"     , id_min      , 1);
    int id_max      ; this->get_parameter_or("init/auto_search_max_id"     , id_max      , 35);

    if ( num_expected>0 ) ROS_INFO("Expected number of Dynamixel is [%d]", num_expected);
    else                  ROS_WARN("\nExpected number of Dynamixel is not set. Free number of Dynamixel is allowed");
    ROS_INFO(" Auto scanning Dynamixel (id range [%d] to [%d]) ...", id_min, id_max);
    auto num_found = ScanDynamixels(id_min, id_max, num_expected, times_retry);
    if( num_found==0 ) { // 見つからなかった場合は初期化失敗で終了
        ROS_ERROR("Dynamixel is not found in USB device [%s]", dyn_comm_.port_name().c_str());
        if ( !is_debug ) throw std::runtime_error("Initialization failed (no dynamixel found)");
    }
    if( num_expected>0 && num_expected!=num_found ) { // 期待数が設定されているときに、見つかった数が期待数と異なる場合は初期化失敗で終了
        ROS_ERROR("Number of Dynamixel is not matched. Expected [%d], but found [%d]. please check & retry", num_expected, num_found);
        if ( !is_debug ) throw std::runtime_error("Initialization failed (number of dynamixel is not matched)");
    }
    ROS_INFO("  ... Finish scanning Dynamixel");

    auto callback_group_subscriber = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_subscriber;
    // Subscriber / Publisherの設定
    if ( num_[SERIES_X] > 0 ) {
        sub_dxl_x_cmds_  = create_subscription<DxlCommandsX>("dynamixel/commands/x", 10, bind(&DynamixelHandler::CallbackCmdsX, this, _1));
        sub_ctrl_x_pwm_  = create_subscription<DynamixelControlXPwm>                 ("dynamixel/command/x/pwm",                   4, bind(&DynamixelHandler::CallbackCmd_X_Pwm, this, _1));
        sub_ctrl_x_cur_  = create_subscription<DynamixelControlXCurrent>             ("dynamixel/command/x/current",               4, bind(&DynamixelHandler::CallbackCmd_X_Current, this, _1));
        sub_ctrl_x_vel_  = create_subscription<DynamixelControlXVelocity>            ("dynamixel/command/x/velocity",              4, bind(&DynamixelHandler::CallbackCmd_X_Velocity, this, _1));
        sub_ctrl_x_pos_  = create_subscription<DynamixelControlXPosition>            ("dynamixel/command/x/position",              4, bind(&DynamixelHandler::CallbackCmd_X_Position, this, _1));
        sub_ctrl_x_epos_ = create_subscription<DynamixelControlXExtendedPosition>    ("dynamixel/command/x/extended_position",     4, bind(&DynamixelHandler::CallbackCmd_X_ExtendedPosition, this, _1));
        sub_ctrl_x_cpos_ = create_subscription<DynamixelControlXCurrentBasePosition> ("dynamixel/command/x/current_base_position", 4, bind(&DynamixelHandler::CallbackCmd_X_CurrentBasePosition, this, _1));
    }
    if ( num_[SERIES_P] > 0) {
        sub_dxl_p_cmds_  = create_subscription<DxlCommandsP>("dynamixel/commands/p", 10, bind(&DynamixelHandler::CallbackCmdsP, this, _1));
        sub_ctrl_p_pwm_  = create_subscription<DynamixelControlPPwm>             ("dynamixel/command/p/pwm",               4, bind(&DynamixelHandler::CallbackCmd_P_Pwm, this, _1));
        sub_ctrl_p_cur_  = create_subscription<DynamixelControlPCurrent>         ("dynamixel/command/p/current",           4, bind(&DynamixelHandler::CallbackCmd_P_Current, this, _1));
        sub_ctrl_p_vel_  = create_subscription<DynamixelControlPVelocity>        ("dynamixel/command/p/velocity",          4, bind(&DynamixelHandler::CallbackCmd_P_Velocity, this, _1));
        sub_ctrl_p_pos_  = create_subscription<DynamixelControlPPosition>        ("dynamixel/command/p/position",          4, bind(&DynamixelHandler::CallbackCmd_P_Position, this, _1));
        sub_ctrl_p_epos_ = create_subscription<DynamixelControlPExtendedPosition>("dynamixel/command/p/extended_position", 4, bind(&DynamixelHandler::CallbackCmd_P_ExtendedPosition, this, _1));
    }
    sub_common_ = create_subscription<DynamixelCommonCmd>("dynamixel/command/common", 4, bind(&DynamixelHandler::CallbackCmd_Common, this, _1));
    sub_status_ = create_subscription<DynamixelStatus>   ("dynamixel/command/status", 4, bind(&DynamixelHandler::CallbackCmd_Status, this, _1));
    sub_goal_   = create_subscription<DynamixelGoal>     ("dynamixel/command/goal",   4, bind(&DynamixelHandler::CallbackCmd_Goal, this, _1));
    sub_gain_   = create_subscription<DynamixelGain>     ("dynamixel/command/gain",   4, bind(&DynamixelHandler::CallbackCmd_Gain, this, _1));
    sub_limit_  = create_subscription<DynamixelLimit>    ("dynamixel/command/limit",  4, bind(&DynamixelHandler::CallbackCmd_Limit, this, _1));

    pub_dxl_states_ = create_publisher<DxlStates>   ("dynamixel/states", 4);
    pub_status_ = create_publisher<DynamixelStatus> ("dynamixel/state/status"  , 4);
    pub_present_= create_publisher<DynamixelPresent>("dynamixel/state/present", 4);
    if ( pub_ratio_["goal"]  ) pub_goal_   = create_publisher<DynamixelGoal>   ("dynamixel/state/goal" , 4);
    if ( pub_ratio_["gain"]  ) pub_gain_   = create_publisher<DynamixelGain>   ("dynamixel/state/gain" , 4);
    if ( pub_ratio_["limit"] ) pub_limit_  = create_publisher<DynamixelLimit>  ("dynamixel/state/limit", 4);
    if ( pub_ratio_["error"] ) pub_error_  = create_publisher<DynamixelError>  ("dynamixel/state/error"  , 4);

    // 状態のreadの前にやるべき初期化
    double init_pa; this->get_parameter_or("init/profile_acceleration", init_pa, 600.0*DEG);
    double init_pv; this->get_parameter_or("init/profile_velocity"    , init_pv, 100.0*DEG);
    for (auto id : id_set_) {
        WriteBusWatchdog (id, 0.0 );
        WriteHomingOffset(id, 0.0 );
        WriteProfileAcc(id, init_pa ); //  設定ファイルからとってこれるようにする
        WriteProfileVel(id, init_pv ); //  設定ファイルからとってこれるようにする
    }

    // 最初の一回は全ての情報をread & publish
    ROS_INFO(" Reading dynamixel states  ...");  

    ROS_INFO(" * present values reading .. "); 
        while ( rclcpp::ok() && SyncReadPresent( list_read_present_ ) < 1.0-1e-6 ) rsleep(50);                                           
    ROS_INFO(" * goal values reading  .. "); 
        while ( rclcpp::ok() && SyncReadGoal ( list_read_goal_  ) < 1.0-1e-6 ) rsleep(50); 
    ROS_INFO(" * gain values reading  .. "); 
        while ( rclcpp::ok() && SyncReadGain ( list_read_gain_  ) < 1.0-1e-6 ) rsleep(50); 
    ROS_INFO(" * limit values reading .. "); 
        while ( rclcpp::ok() && SyncReadLimit( list_read_limit_ ) < 1.0-1e-6 ) rsleep(50); 
    ROS_INFO(" * hardware error reading .. "); 
        while ( rclcpp::ok() && SyncReadHardwareErrors() < 1.0-1e-6 ) rsleep(50); // 最後にやる
    ROS_INFO(" * other status values reading  ..");
    for (auto id : id_set_) {
        tq_mode_[id] = ReadTorqueEnable(id);
        op_mode_[id] = ReadOperatingMode(id);
        dv_mode_[id] = ReadDriveMode(id);
        limit_w_[id] = limit_r_[id];
        gain_w_[id] = gain_r_[id];
        goal_w_[id] = goal_r_[id];
    }
    ROS_INFO("  ... states reading done");

    BroadcastState_Status();
    BroadcastState_Limit();
    BroadcastState_Gain();  
    BroadcastState_Goal();   
    BroadcastState_Error(); 

    for ( auto id : id_set_ ) if ( abs(init_pa - goal_r_[id][PROFILE_ACC]) > 0.1 ) ROS_WARN("\nProfile acceleration is not set correctly [%d], your setting [%f], but [%f]", id, init_pa, goal_r_[id][PROFILE_ACC]);
    for ( auto id : id_set_ ) if ( abs(init_pv - goal_r_[id][PROFILE_VEL]) > 0.1 ) ROS_WARN("\nProfile velocity is not set correctly [%d], your setting [%f], but [%f]", id, init_pv, goal_r_[id][PROFILE_VEL]);

    // 状態のreadの後にやるべき初期化
    ROS_INFO( " Initializing dynamixel state  ...");
    bool do_clean_hwerr; this->get_parameter_or("init/hardware_error_auto_clean", do_clean_hwerr, true);
    bool do_torque_on  ; this->get_parameter_or("init/torque_auto_enable"       , do_torque_on  , true);
    for (auto id : id_set_) {
        if ( do_clean_hwerr ) ClearHardwareError(id); // 現在の状態を変えない
        if ( do_torque_on )   TorqueOn(id);           // 現在の状態を変えない
    } ROS_INFO( "  ... state initialization done ");

    ROS_INFO( "..... DynamixelHandler is initialized");
}

using std::chrono::system_clock;
using std::chrono::duration_cast;
using std::chrono::microseconds;

void DynamixelHandler::MainLoop(){
    static int cnt = -1; cnt++;
    static float rtime=0, wtime=0, num_st_suc_p=1, num_st_suc_f=1, num_st_read=1;

/* 処理時間時間の計測 */ auto wstart = system_clock::now();
    //* topicをSubscribe & Dynamixelへ目標角をWrite
    SyncWriteGoal(list_write_goal_);
    list_write_goal_.clear();
    SyncWriteGain(list_write_gain_);
    list_write_gain_.clear();
/* 処理時間時間の計測 */ wtime += duration_cast<microseconds>(system_clock::now()-wstart).count() / 1000.0;

    //* present value について read する情報を決定
    static const auto& r = pub_ratio_present_; //長いので省略
    list_read_present_.clear();
    if (r[PRESENT_PWM          ] && cnt % r[PRESENT_PWM          ] == 0) list_read_present_.insert(PRESENT_PWM          );
    if (r[PRESENT_CURRENT      ] && cnt % r[PRESENT_CURRENT      ] == 0) list_read_present_.insert(PRESENT_CURRENT      );
    if (r[PRESENT_VELOCITY     ] && cnt % r[PRESENT_VELOCITY     ] == 0) list_read_present_.insert(PRESENT_VELOCITY     );
    if (r[PRESENT_POSITION     ] && cnt % r[PRESENT_POSITION     ] == 0) list_read_present_.insert(PRESENT_POSITION     );
    if (r[VELOCITY_TRAJECTORY  ] && cnt % r[VELOCITY_TRAJECTORY  ] == 0) list_read_present_.insert(VELOCITY_TRAJECTORY  );
    if (r[POSITION_TRAJECTORY  ] && cnt % r[POSITION_TRAJECTORY  ] == 0) list_read_present_.insert(POSITION_TRAJECTORY  );
    if (r[PRESENT_INPUT_VOLTAGE] && cnt % r[PRESENT_INPUT_VOLTAGE] == 0) list_read_present_.insert(PRESENT_INPUT_VOLTAGE);
    if (r[PRESENT_TEMPERATURE  ] && cnt % r[PRESENT_TEMPERATURE  ] == 0) list_read_present_.insert(PRESENT_TEMPERATURE   );

/* 処理時間時間の計測 */ auto rstart = system_clock::now();
    if ( loop_rate_ && cnt % loop_rate_ == 0 ) CheckDynamixels(); // Statusの確認
    //* Dynamixelから状態Read & topicをPublish
    if ( !list_read_present_.empty() ) // list_read_present_が空でない場合のみ実行
    if ( pub_ratio_["present"] && cnt % pub_ratio_["present"] == 0 ) {// pub_ratio_["present"]の割合で実行
        double rate_suc_st = SyncReadPresent(list_read_present_);
        num_st_read++;
        num_st_suc_p += rate_suc_st > 0.0;
        num_st_suc_f += rate_suc_st > 1.0-1e-6;
        BroadcastState_Present();
    }
    if ( pub_ratio_["error"] && cnt % pub_ratio_["error"] == 0 ) { // pub_ratio_["error"]の割合で実行
        double rate_suc_err = SyncReadHardwareErrors();
        if ( rate_suc_err>0.0) BroadcastState_Error();
    }
    if ( pub_ratio_["limit"] && cnt % pub_ratio_["limit"] == 0 ) { // pub_ratio_["limit"]
        double rate_suc_lim = SyncReadLimit(list_read_limit_); // 処理を追加する可能性を考えて，変数を別で用意する冗長な書き方をしている．
        if ( rate_suc_lim >0.0 ) BroadcastState_Limit();
    }
    if (pub_ratio_["gain"] && cnt %pub_ratio_["gain"] == 0 ) { //pub_ratio_["gain"]
        double rate_suc_gain = SyncReadGain(list_read_gain_);
        if ( rate_suc_gain>0.0 ) BroadcastState_Gain();
    }
    if (pub_ratio_["goal"] && cnt %pub_ratio_["goal"] == 0 ) { //pub_ratio_["goal"]
        double rate_suc_goal = SyncReadGoal(list_read_goal_);
        if ( rate_suc_goal>0.0 ) BroadcastState_Goal();
    }
/* 処理時間時間の計測 */ rtime += duration_cast<microseconds>(system_clock::now()-rstart).count() / 1000.0;

    //* デバック
    if ( ratio_mainloop_ !=0 )
    if ( cnt % ratio_mainloop_ == 0) {
        float partial_suc = 100*num_st_suc_p/num_st_read; 
        float full_suc = 100*num_st_suc_f/num_st_read;
        char msg[100]; sprintf(msg, "Loop [%d]: write=%.2fms read=%.2fms(p/f=%3.0f%%/%3.0f%%)",
                               cnt, wtime/ratio_mainloop_, rtime/num_st_read, partial_suc, full_suc);
        if (full_suc < 80) ROS_ERROR("%s", msg); else if (partial_suc < 99) ROS_WARN("%s", msg); else ROS_INFO("%s", msg);
        wtime = 0.0; /* mainloopで行われてる処理の計測時間を初期化 */
    }
    if ( cnt % max({(int)loop_rate_, (int)ratio_mainloop_, 10}) == 0)
        rtime = num_st_suc_p = num_st_suc_f = num_st_read=0.00001; /* stateのreadの周期で行われてる処理の初期化 */ 

	fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する
}

DynamixelHandler::~DynamixelHandler(){
    ROS_INFO( "Terminating DynamixelHandler ...");
    bool do_torque_off; get_parameter_or("term/torque_auto_disable", do_torque_off, true);
    if ( do_torque_off ) for ( auto id : id_set_ ) TorqueOff(id);
    StopDynamixels();
    ROS_INFO( "  ... DynamixelHandler is terminated");
}

#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
	/*Initialization*/
    auto node = std::make_shared<DynamixelHandler>();
    /*Mainloop*/
    auto timer_ = node.get()->create_wall_timer(
          1.0s/(node->loop_rate_)
        , bind(&DynamixelHandler::MainLoop, node.get())
    ); // 変数に保存する必要あり
    /*Interruption*/
    auto executor = rclcpp::executors::MultiThreadedExecutor::make_unique();
    executor->add_node(node);
    executor->spin();
    /*Termination*/
    node.reset(); // rclcpp::shutdown() の前に呼ぶ必要あり
    rclcpp::shutdown();
    return 0;
}