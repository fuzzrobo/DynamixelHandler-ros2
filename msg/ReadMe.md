# Custom msg design

## Abstract
   プログラムとコマンドライン両方での使用する場合の利便性を考慮して設計を行った．
   コマンドラインで使用する場合は，できるだけ細かく分割されたmsgの方が，閲覧や送信が楽である．
   しかし，プログラムで使用する場合は，pub/subするトピック数を減らしたいため，全ての情報を一つのmsgにまとめることが有効である．
   そのため，コマンドラインで使用する細かいmsgを用意し，それらを複数まとめた大きなmsgをプログラムで使用することとした．

 - Usage in program
   - read: `/dynamixel/states` topic
   - write: `/dynamixel/commands/x` topic (for X series), `/dynamixel/commands/p` topic (for P series)
 - Usage in command line
   - read: `/dynamixel/state/...` topics, `/dynamixel/debug` topic
   - write: `/dynamixel/command/...` topics

-----

## Topic list 
全てのtopicのリストを示す．

#### プログラムから使う想定もの

   - `/dynamixel/states` : すべての状態をまとめたmsg
   - `/dynamixel/commands/x` : Xシリーズのコマンドをまとめたmsg
   - `/dynamixel/commands/p` : Pシリーズのコマンドをまとめたmsg
   - `/dynamixel/ex_port/write` : 外部ポートへの書き込み・設定を行うmsg
   - `/dynamixel/ex_port/read` : 外部ポートの読み取りを行うmsg
  
#### コマンドラインから使う想定のもの

   デバック用
   - `/dynamixel/debug` : サーボが動かないときにに確認したい情報をまとめたもの

   状態確認用
   - `/dynamixel/state/status` : サーボの状態を示すmsg
   - `/dynamixel/state/present` : 現在の値を示すmsg
   - `/dynamixel/state/goal` : 目標値を示すmsg
   - `/dynamixel/state/gain` : ゲインを示すmsg
   - `/dynamixel/state/limit` : 制限値を示すmsg
   - `/dynamixel/state/error` : エラーを示すmsg
   - `/dynamixel/state/extra` : その他の情報を示すmsg

   コマンド送信用
   - `/dynamixel/command/common` : トルクのオンオフ・エラー解除・IDの追加削除などを行う
   - `/dynamixel/command/x/pwm_control` : pwm制御モードでの指令を送る
   - `/dynamixel/command/x/current_control` : 電流制御モードでの指令を送る
   - `/dynamixel/command/x/velocity_control` : 速度制御モードでの指令を送る
   - `/dynamixel/command/x/position_control` : 位置制御モードでの指令を送る
   - `/dynamixel/command/x/extended_position_control` :　拡張位置制御モードでの指令を送る
   - `/dynamixel/command/x/current_base_position_control` :　電流制限付き位置制御モードでの指令を送る
   - `/dynamixel/command/status` : サーボの状態を変更する
   - `/dynamixel/command/goal` : サーボの目標値を変更する
   - `/dynamixel/command/gain` : ゲインを変更する
   - `/dynamixel/command/limit` : 制限値を変更する
   - `/dynamixel/command/extra` : その他の情報を変更する

##### Pシリーズを併用する場合，以下が追加される．
   - `/dynamixel/command/p/pwm_control` : pwm制御モードでの指令を送る
   - `/dynamixel/command/p/current_control` : 電流制御モードでの指令を送る
   - `/dynamixel/command/p/velocity_control` : 速度制御モードでの指令を送る
   - `/dynamixel/command/p/position_control` : 位置制御モードでの指令を送る
   - `/dynamixel/command/p/extended_position_control` : 拡張位置制御モードでの指令を送る

-----

## How to use

### 情報をSubscribe(read)する場合
#### プログラムでの使用
```cpp
#include "dynamixel_handler/msg/dxl_states.hpp"
/// ...
/// 略, 動くコードは example を参照のこと
/// ...
dynamixel_handler::msg::DxlStates msg; //すべての状態が確認できる
// subscribe した status の確認, 単に表示するだけ
for (size_t i = 0; i < msg.status.id_list.size(); i++) {
   printf("servo [%d], torque %s, has %s, ping is %s, mode is %s\n", 
      msg.status.id_list[i],
      msg.status.torque[i] ? "on" : "off",
      msg.status.error[i] ? "error" : "no error",
      msg.status.ping[i] ? "response" : "no response",
      msg.status.mode[i]
   ); 
}
// subscribe した present value を変数に格納, present valueはすべての要素があるとは限らないので確認が必要
auto p = msg.present;
for (size_t i=0; i < p.id_list.size(); i++) {
   auto id = p.id_list[i];
   if ( !p.current_ma.empty()     ) cur_map[id] = p.current_ma[i];
   if ( !p.velocity_deg_s.empty() ) vel_map[id] = p.velocity_deg_s[i];
   if ( !p.position_deg.empty()   ) pos_map[id] = p.position_deg[i];
}
```

#### コマンドラインでの使用

```yaml  
$ ros2 topic echo --flow-style /dynamixel/debug #debug用, これがあると便利
status: # DynamixelStatus型
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false]
   error: [false, false, false, false]
   ping: [true, true, true, true]
   mode: ['position', 'velocity', 'current', 'velocity']
current_ma: # DynamixelDebugElement型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]
velocity_deg_s: # DynamixelDebugElement型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]
position_deg: # DynamixelDebugElement型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]

$ ros2 topic echo --flow-style /dynamixel/state/limit #  DynamixelLimit型
id_list: [1,2,3,4]
temperature_limit_degc: [0.0, 0.0, 0.0, 0.0]
max_voltage_limit_v: [0.0, 0.0, 0.0, 0.0]
min_voltage_limit_v: [0.0, 0.0, 0.0, 0.0]
pwm_limit_percent: [0.0, 0.0, 0.0, 0.0]
current_limit_ma: [0.0, 0.0, 0.0, 0.0]
acceleration_limit_deg_ss: [0.0, 0.0, 0.0, 0.0]
velocity_limit_deg_s: [0.0, 0.0, 0.0, 0.0]
max_position_limit_deg: [0.0, 0.0, 0.0, 0.0]
min_position_limit_deg: [0.0, 0.0, 0.0, 0.0]

$ ros2 topic echo --flow-style /dynamixel/state/present #  DynamixelPresent型
id_list: [1,2,3,4]
pwm_pulse: [0.0, 0.0, 0.0, 0.0]
current_ma: [0.0, 0.0, 0.0, 0.0]
velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
position_deg: [0.0, 0.0, 0.0, 0.0]
vel_trajectory_deg_s: [0.0, 0.0, 0.0, 0.0]
pos_trajectory_deg: [0.0, 0.0, 0.0, 0.0]
input_voltage_v: []
temperature_degc: []

# ... 略
```

### 指令をPublish(write)する場合

#### プログラムでの使用
```cpp
#include "dynamixel_handler/msg/dxl_commands_x.hpp"
/// ...
/// 略, 動くコードは example を参照のこと
/// ...
dynamixel_handler::msg::DxlCommandsX cmd;

// id = 1,2,3 のサーボを **torque_on**.
cmd.common.command = "torque_on";
for (int i=1; i<=3; i++) cmd.common.id_list.push_back(i);
pub_dxl_cmd_->publish( cmd );

// id:1 のサーボを電流制御モードで50degに移動
cmd.current_base_position_control.id_list.push_back(1);
cmd.current_base_position_control.position_deg.push_back(50);
pub_dxl_cmd_->publish( cmd );

// id:2 のサーボのdゲインを50に設定
cmd.gain.id_list.push_back(2); 
cmd.gain.position_d_gain_pulse.push_back(50.0); 
pub_dxl_cmd_->publish( cmd );

// id:3 のサーボのgoal値を直接書き換え
cmd.goal.id_list.push_back(3);
cmd.goal.position_deg.push_back(100.0);//  goal値への書き込みはcontrol mode次第で有効
pub_dxl_cmd_->publish( cmd );

// より実践的には以下のように使う
std::map<uint8_t, std::tuple<float, float>> target_map = {
   {1, {50.0, 0.0}},
   {2, {100.0, 0.0}},
   {3, {150.0, 0.0}},
};
for ( auto [id, target] : target_map ) {
   auto [pos, cur] = target;
   cmd.current_base_position_control.id_list.push_back(id);
   cmd.current_base_position_control.position_deg.push_back(pos);
   cmd.current_base_position_control.current_ma.push_back(cur);
   cmd.current_base_position_control.profile_vel_deg_s.push_back(100.0);
   cmd.current_base_position_control.profile_acc_deg_ss.push_back(100.0);
}
pub_dxl_cmd_->publish( cmd );
```

#### コマンドラインでの使用
```bash
ros2 topic pub /dynamixel/command/common dynamixel_handler/msg/DynamixelCommonCmd \
"command: 'torque_on'
id_list: [1,2,3,4]" -1

ros2 topic pub /dynamixel/command/current_base_position_control dynamixel_handler/msg/DynamixelControlXCurrentPosition \
"id_list: [1,2,3,4]
current_ma: [0.0, 0.0, 0.0, 0.0]
position_deg: [0.0, 0.0, 0.0, 0.0]
rotation: [0.0, 0.0, 0.0, 0.0]
profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]" -1

# ... 略
```

-----

## Topic detail

### `/dynamixel/states` (`dynamixel_handler::msg::DxlStates` type)
```yaml 
$ ros2 topic echo --flow-style /dynamixel/states #このtopicはコマンドラインから見る想定ではない．
stamp: 0000
status: # DynamixelStatus型, pub_ratio/status に一回 read される．．
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false]
   error: [false, false, false, false]
   ping: [true, true, true, true]
   mode: ['position', 'velocity', 'current', 'velocity']
present: # DynamixelPresent型, pub_ratio/present.~ に一回 read され，1要素でも読み取ったら埋める
   id_list: [1, 2, 3, 4]
   pwm_pulse: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.pwm に一回 read される．
   current_ma: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.current に一回 read される．
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.velocity に一回 read される．
   position_deg: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.position に一回 read される．
   vel_trajectory_deg_s: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.vel_trajectory に一回 read される．
   pos_trajectory_deg: [0.0, 0.0, 0.0, 0.0] # pub_ratio/present.pos_trajectory に一回 read される．
   input_voltage_v: [] # pub_ratio/present.input_voltage に一回 read される．
   temperature_degc: [] # pub_ratio/present.temperature に一回 read される．
goal: # DynamixelGoal型, pub_ratio/goalに一回 read され，読み取ったら埋める
   id_list: [1, 2, 3, 4]
   pwm_pulse: [0.0, 0.0, 0.0, 0.0]
   current_ma: [0.0, 0.0, 0.0, 0.0]
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   position_deg: [0.0, 0.0, 0.0, 0.0]
limit: # DynamixelLimit型, pub_ratio/limitに一回 read され，読み取りに成功したら埋める．
   id_list: []
   temperature_limit_degc: []
   max_voltage_limit_v: []
   min_voltage_limit_v: []
   pwm_limit_percent: []
   current_limit_ma: []
   acceleration_limit_deg_ss: []
   velocity_limit_deg_s: []
   max_position_limit_deg: []
   min_position_limit_deg: []
gain: # DynamixelGain型, pub_ratio/gainに一回 read され，読み取りに成功したら埋める．
   id_list: [1, 2, 3, 4]
   velocity_i_gain_pulse: [0, 0, 0, 0]
   velocity_p_gain_pulse: [0, 0, 0, 0]
   position_d_gain_pulse: [0, 0, 0, 0]
   position_i_gain_pulse: [0, 0, 0, 0]
   position_p_gain_pulse: [0, 0, 0, 0]
   feedforward_2nd_gain_pulse: [0, 0, 0, 0]
   feedforward_1st_gain_pulse: [0, 0, 0, 0]
error: # DynamxielError型, pub_ratio/errorに一回 read され，読み取りに成功したら埋める．
   id_list: [1, 2, 3, 4]
   input_voltage: [false, false, false, false]
   motor_hall_sensor: [false, false, false, false]
   overheating: [false, false, false, false]
   motor_encoder: [false, false, false, false]
   electronical_shock: [false, false, false, false]
   overload: [false, false, false, false]
extra: # DynamixelExtra型, 未実装
   id_list: []
   model: []
   firmware_version: []
   protocol_version: []
   drive_mode:
      torque_on_by_goal_update: []
      profile_configuration: []
      reverse_mode: []
   shutdown:
      overload_error: []
      electrical_shock_error: []
      motor_encorder_error: []
      overheating_error: []
      input_voltage_error: []
   restore_configuration:
      ram_restore: []
      startup_torque_on: []
   homing_offset_deg: []
   return_delay_time_us: []
   bus_watchbdog_ms: []
   led: []
   shadow_id: []
   moving_threshold_deg_s: []
   status_return_level: []
   moving_status:
      velocity_profile: []
      following_error: []
      profile_ongoing: []
      in_posision: []
   realtime_tick_us: []
   moving: []
   registered_instruction: []
```

### `/dynamixel/commands/x` (`dynamixel_handler::msg::DxlCommandsX` type)
```yaml
$ ros2 topic echo --flow-style /dynamixel/commands/x #このtopicはコマンドラインから送る想定ではない．
common: #DynamixelCommonCmd型
   command: "torque_on" #要素が1つの時は全idに適用
   id_list: [1,2,3,4]
pwm_control: # DynamixelControlXPwm型
   id_list: [1,2,3,4]
   pwm_percent: [0.0, 0.0, 0.0, 0.0]
current_control: # DynamixelControlXCurrent型
   id_list: [1,2,3,4]
   current_ma: [0.0, 0.0, 0.0, 0.0]
velocity_control: # DynamixelControlXVelocity型
   id_list: [1,2,3,4]
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
position_control: # DynamixelControlXPosition型
   id_list: [1,2,3,4]
   position_deg: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
extended_position_control: # DynamixelControlXExtendedPosition型
   id_list: [1,2,3,4]
   position_deg: [0.0, 0.0, 0.0, 0.0]
   rotation: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
current_base_position_control: # DynamixelControlXCurrentPosition型
   id_list: [1,2,3,4]
   current_ma: [0.0, 0.0, 0.0, 0.0]
   position_deg: [0.0, 0.0, 0.0, 0.0]
   rotation: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
status: # DynamixelStatus型
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false] # torque_onコマンド, torque_offコマンドと同等
   error: [false, false, false, false] # clear_errorコマンドと同等
   ping: [true, true, true, true] # add_id コマンド, remove_id コマンドと同等
   mode: ['position', 'velocity', 'current', 'velocity'] # 各control系のコマンドと同等
goal: # DynamixelGoal型
   id_list: [1,2,3,4]
   pwm_pulse: [0.0, 0.0, 0.0, 0.0]
   current_ma: [0.0, 0.0, 0.0, 0.0]
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   position_deg: [0.0, 0.0, 0.0, 0.0]
gain: # DynamixelGain型
   id_list: [1,2,3,4]
   velocity_i_gain_pulse: [0, 0, 0, 0]
   velocity_p_gain_pulse: [0, 0, 0, 0]
   position_d_gain_pulse: [0, 0, 0, 0]
   position_i_gain_pulse: [0, 0, 0, 0]
   position_p_gain_pulse: [0, 0, 0, 0]
   feedforward_2nd_gain_pulse: [0, 0, 0, 0]
   feedforward_1st_gain_pulse: [0, 0, 0, 0]
limit: # DynamixelLimit型
   id_list: [1,2,3,4]
   temperature_limit_degc: [0.0, 0.0, 0.0, 0.0]
   max_voltage_limit_v: [0.0, 0.0, 0.0, 0.0]
   min_voltage_limit_v: [0.0, 0.0, 0.0, 0.0]
   pwm_limit_percent: [0.0, 0.0, 0.0, 0.0]
   current_limit_ma: [0.0, 0.0, 0.0, 0.0]
   acceleration_limit_deg_ss: [0.0, 0.0, 0.0, 0.0]
   velocity_limit_deg_s: [0.0, 0.0, 0.0, 0.0]
   max_position_limit_deg: [0.0, 0.0, 0.0, 0.0]
   min_position_limit_deg: [0.0, 0.0, 0.0, 0.0]
extra:
   id_list: []
   model: []
   firmware_version: []
   protocol_version: []
   drive_mode:
      torque_on_by_goal_update: []
      profile_configuration: []
      reverse_mode: []
   shutdown:
      overload_error: []
      electrical_shock_error: []
      motor_encorder_error: []
      overheating_error: []
      input_voltage_error: []
   restore_configuration:
      ram_restore: []
      startup_torque_on: []
   homing_offset_deg: []
   return_delay_time_us: []
   bus_watchbdog_ms: []
   led: []
   shadow_id: []
   moving_threshold_deg_s: []
   status_return_level: []
   moving_status:
      velocity_profile: []
      following_error: []
      profile_ongoing: []
      in_posision: []
   realtime_tick_us: []
   moving: []
   registered_instruction: []      
```

### `/dynamixel/commands/p` (`dynamixel_handler::msg::DxlCommandsP` type)
略

### その他のコマンドライン用トピックとその型定義

#### `/dynamixel/command/common` (`DynamixelCommonCmd` type) 
トルクのオンオフなどのコマンドを送るtopic
   ```yml
   string   command
   uint16[]  id_list
   # === high lebel commnads : ユーザの利用を想定 ===
   string CLEAR_ERROR ="clear_error" # ("CE"): ハードウェアエラー(ex. overload)をrebootによって解除する．
                                     # homing offsetを用いて現在角がジャンプしないように調整する．
   string TORQUE_OFF ="torque_off" # ("TOFF"): トルクをdisableにする．
   string TORQUE_ON  ="torque_on"  # ("TON") : 安全にトルクをenableにする．目標姿勢を現在姿勢へ一致させ，速度を0にする．
   string REMOVE_ID  ="remove_id"  # ("RMID"): 指定したIDのサーボを認識リストから削除する．
   string ADD_ID     ="add_id"     # ("ADID"): 指定したIDのサーボを認識リストに追加する．
   # === low level commands : 開発者向け ===
   string RESET_OFFSET="reset_offset" # : homing_offset アドレスに 0 を書き込む．
   string ENABLE ="enable"  # : torque enable アドレスに true を書き込む．
   string DISABLE="disable" # : torque enable アドレスに false を書き込む．
   string REBOOT ="reboot"  # : reboot インストラクションを送る．
   ```
各コマンドの内容はmsgの定数として定義されており，プログラム内から扱う場合，以下の様に記述できる．
   ```cpp
   dynamixel_handler::msg::DxlCommandsX cmd;
   cmd.common.command = msg.TORQUE_ON; // typoはコンパイラが教えてくれる．
   // cmd.common.command = "torque_on"; // もちろんこれもOKだが，typoのリスクがある．
   ```
また，括弧内はalias．すなわち，`command="clear_error"`とするのと`command="CE"`とするのは同じ．

#### `/dynamixel/command/x/pwm_control` (`DynamixelControlXPwm` type)
XシリーズをPWM制御モードで動かすためのtopic
   ```yml
   uint16[] id_list
   float64[] pwm_percent
   ```

#### `/dynamixel/command/x/current_control` (`DynamixelControlXCurrent` type)
Xシリーズを電流制御モードで動かすためのtopic
   ```yml
   uint16[] id_list
   float64[] current_mA
   ```


#### `/dynamixel/command/x/velocity_control` (`DynamixelControlXVelocity` type)
Xシリーズを速度制御モードで動かすためのtopic
   ```yml
   uint16[] id_list
   float64[] velocity_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `/dynamixel/command/x/position_control` (`DynamixelControlXPosition` type)
Xシリーズを位置制御モードで動かすためのtopic
   ```yml
   uint16[] id_list
   float64[] position_deg
   float64[] profile_vel_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `/dynamixel/command/x/extended_position_control` (`DynamixelControlXExtendedPosition` type)
Xシリーズを拡張位置制御モードで動かすためのtopic
   ```yml
   uint16[] id_list
   float64[] position_deg
   float64[] rotation # optional, 256までの回転数を指定できる
   float64[] profile_vel_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `/dynamixel/command/x/current_position _control` (`DynamixelControlXCurrentPosition` type)
Xシリーズを電流制限付き位置制御モードで動かすためのtopic
   ```yml
   uint16[] id_list
   float64[] current_ma
   float64[] position_deg
   float64[] rotation # optional, 256までの回転数を指定できる
   float64[] profile_vel_deg_s
   float64[] profile_acc_deg_ss
   ```

#### `/dynamixel/command/status` (`DynamixelStatus` type)
   ```yml
   uint16[] id_list
   bool[] torque
   bool[] error
   bool[] ping
   string[] mode
   ```

#### `/dynamixel/command/goal` (`DynamixelGoal` type)
   ```yml
   uint16[] id_list
   float64[] pwm_pulse
   float64[] current_ma
   float64[] velocity_deg_s
   float64[] profile_acc_deg_ss
   float64[] profile_vel_deg_s
   float64[] position_deg
   ```

#### `/dynamixel/command/gain` (`DynamixelGain` type)
   ```yml
   uint16[] id_list
   float64[] velocity_i_gain_pulse
   float64[] velocity_p_gain_pulse
   float64[] position_d_gain_pulse  
   float64[] position_i_gain_pulse
   float64[] position_p_gain_pulse
   float64[] feedforward_2nd_gain_pulse
   float64[] feedforward_1st_gain_pulse
   ```
#### `/dynamixel/command/limit` (`DynamixelLimit` type)
   ```yml
   uint16[] id_list
   float64[] temperature_limit_degc
   float64[] max_voltage_limit_v
   float64[] min_voltage_limit_v
   float64[] pwm_limit_percent
   float64[] current_limit_ma
   float64[] acceleration_limit_deg_ss
   float64[] velocity_limit_deg_s
   float64[] max_position_limit_deg
   float64[] min_position_limit_deg
   ```

---
---
---

## external port に関して
こいつだけ, XH540シリーズだけで使える機能なので，独立させる．
```cpp
// ID: 1 のサーボのポート1と2にはLEDが接続されている
constexpr int ID_LIGHT = 1;
constexpr int PORT_LIGHT1 = 1;
constexpr int PORT_LIGHT2 = 2;
// ID: 1 のサーボのポート3はマグネットセンサが接続されている
constexpr int ID_MAGNET = 2;
constexpr int PORT_MAGNET = 3;
// 使い方1 書き込み
ExternalPort msg;
msg.set__id_list({ID_LIGHT            , ID_LIGHT            });
msg.set__port   ({PORT_LIGHT1         , PORT_LIGHT2         });
msg.set__mode   ({msg.MODE_DIGITAL_OUT, msg.MODE_DIGITAL_OUT});
msg.set__data   ({level>0 ? 1 : 0, level>1 ? 1 : 0});
msg.id_list.push_back(ID_MAGNET);
msg.port.push_back(PORT_MAGNET);
msg.mode.push_back(msg.MODE_ANALOG_IN);
pub_ex_port_->publish( msg );

// 使い方2 読み込み
const int val_magnet = 0;
for ( size_t i = 0; i < msg.id_list.size(); i++ ) {
   if ( msg.id_list[i] != ID_MAGNET ) continue;
   if ( msg.port[i] != PORT_MAGNET ) continue;
   val_magnet = msg.data[i];
}
printf("magnet sensor is %d\n", val_magnet);
```
`dynamixel_handler::msg::ExternalPort` の中身
```yaml
$ ros2 topic echo --flow-style /dynamixel/ex_port/write # ex_port型
stamp: 0000
id_list: [1, 1, 2]
port: [1, 2, 3]
mode: ["d_out", "d_out", "a_in"]
data: [1, 0, 0]

$ ros2 topic echo --flow-style /dynamixel/ex_port/read # ex_port型
stamp: 0000
id_list: [1, 1, 2]
port: [1, 2, 3]
mode: ["d_out", "d_out", "a_in"]
data: [1, 0, 100]
---
stamp: 0001
id_list: [2]
port: [3]
mode: ["a_in"]
data: [200]
---
```
高周期でreadする可能性もあるし，readするしないを決めるための方法が必要かもしれない．

取りあえず，ex_port/writeトピックで触れてないポートは，readしないことにしよう．
そして，a_in/d_in系の書き込みをしたものについては，常時read, それ以外は，ex_port/writeトピックの直後だけreadする．
