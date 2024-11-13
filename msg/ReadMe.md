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

## topic list 
全てのtopicのリストを示す．
```bash
### publish ###
# コマンドライン用
/dynamixel/debug #debug型 <- 動かなかったときのデバッグ用
/dynamixel/state/status #status型
/dynamixel/state/present #present型
/dynamixel/state/goal #goal型
/dynamixel/state/gain #gain型
/dynamixel/state/limit #limit型
/dynamixel/state/error #error型
/dynamixel/state/extra #extra型
# プログラム用
/dynamixel/states # states型
   # = status型 + present型 + goal型 + mode型 + limit型 + gain型

### subscribe ###
# コマンドライン用
/dynamixel/command/common # common_cmd型
/dynamixel/command/pwm_control # cnt_x_pwm型
/dynamixel/command/position_control # cnt_x_pos型
/dynamixel/command/velocity_control # cnt_x_vel型
/dynamixel/command/current_control # cnt_x_cur型
/dynamixel/command/extended_position_control # cnt_x_e_pos型
/dynamixel/command/current_base_position_control # cnt_x_c_pos型
/dynamixel/command/status # status型
/dynamixel/command/goal # goal型
/dynamixel/command/gain # gain型
/dynamixel/command/limit # limit型
/dynamixel/command/extra # extra型
# プログラム用
/dynamixel/commands #commands_x型
   # = common_cmd型 + cnt_x_pos型 + cnt_x_vel型 + cnt_x_cur型 + cnt_x_c_pos 
   # + cnt_x_e_pos型 + mode型 + goal型 + limit型 + gain型

### pub/sub ###
/dynamixel/ex_port/read # ex_port型
/dynamixel/ex_port/write # ex_port型
```
---
Pシリーズを併用する場合
```bash
### publish
# 同じ

### subscribe
# コマンドライン用
/dynamixel/command/common # common_cmd型
/dynamixel/command/x/pwm_control # cnt_x_pwm型
/dynamixel/command/x/position_control # cnt_x_pos型
/dynamixel/command/x/velocity_control # cnt_x_vel型
/dynamixel/command/x/current_control # cnt_x_cur型
/dynamixel/command/x/extended_position_control # cnt_x_e_pos型
/dynamixel/command/x/current_base_position_control # cnt_x_c_pos型
/dynamixel/command/p/pwm_control # cnt_x_pwm型
/dynamixel/command/p/position_control # cnt_p_pos型
/dynamixel/command/p/velocity_control # cnt_p_vel型
/dynamixel/command/p/current_control # cnt_p_cur型
/dynamixel/command/p/extended_position_control # cnt_p_e_pos型
/dynamixel/command/status # status型
/dynamixel/command/goal # goal型
/dynamixel/command/gain # gain型
/dynamixel/command/limit # limit型
/dynamixel/command/extra # extra型
# プログラム用
/dynamixel/commands/p #commands_p型
/dynamixel/commands/x #commands_x型
/dynamixel/ex_port/write # los_ex_port型
```
---

## 詳細

### readする情報
#### プログラムでの使用
以下のように利用する想定
```cpp
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
`dynamixel_handler::msg::DxlStates` の中身
```yaml 
$ ros2 topic echo --flow-style /dynamixel/states #このtopicはコマンドラインから見る想定ではない．
# 全てのfieldがid_listを持つ必要がある．∵コマンドラインから扱うため
stamp: 0000
status: # status型, read_ratio/status の周期で読み取る．読み取り周期に関わらず常に埋める．
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false]
   error: [false, false, false, false]
   ping: [true, true, true, true]
   mode: ['position', 'velocity', 'current', 'velocity']
present: #present型, read_ratio/present.~ の周期で読み取り，1要素でも読み取ったら埋める
   id_list: [1, 2, 3, 4]
   pwm_pulse: [0.0, 0.0, 0.0, 0.0] # read_ratio/present.pwm の周期で読み取る．
   current_ma: [0.0, 0.0, 0.0, 0.0] # read_ratio/present.current の周期で読み取る．
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0] # read_ratio/present.velocity の周期で読み取る．
   position_deg: [0.0, 0.0, 0.0, 0.0] # read_ratio/present.position の周期で読み取る．
   vel_trajectory_deg_s: [0.0, 0.0, 0.0, 0.0] # read_ratio/present.vel_trajectory の周期で読み取る．
   pos_trajectory_deg: [0.0, 0.0, 0.0, 0.0] # read_ratio/present.pos_trajectory の周期で読み取る．
   input_voltage_v: [] # read_ratio/present.input_voltage の周期で読み取る．
   temperature_degc: [] # read_ratio/present.temperature の周期で読み取る．
goal: #goal型, read_ratio/goalの周期で読み取り，読み取ったら埋める
   id_list: [1, 2, 3, 4]
   pwm_pulse: [0.0, 0.0, 0.0, 0.0]
   current_ma: [0.0, 0.0, 0.0, 0.0]
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   position_deg: [0.0, 0.0, 0.0, 0.0]
limit: #limit型, read_ratio/limitの周期で読み取り，読み取ったら埋める
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
gain: #gain型, read_ratio/gainの周期で読み取り，読み取ったら埋める
   id_list: [1, 2, 3, 4]
   velocity_i_gain_pulse: [0, 0, 0, 0]
   velocity_p_gain_pulse: [0, 0, 0, 0]
   position_d_gain_pulse: [0, 0, 0, 0]
   position_i_gain_pulse: [0, 0, 0, 0]
   position_p_gain_pulse: [0, 0, 0, 0]
   feedforward_2nd_gain_pulse: [0, 0, 0, 0]
   feedforward_1st_gain_pulse: [0, 0, 0, 0]
error: #error型, read_ratio/errorの周期で読み取り，読み取ったら埋める
   id_list: [1, 2, 3, 4]
   input_voltage: [false, false, false, false]
   motor_hall_sensor: [false, false, false, false]
   overheating: [false, false, false, false]
   motor_encoder: [false, false, false, false]
   electronical_shock: [false, false, false, false]
   overload: [false, false, false, false]
extra: # ここはまだ詳細未定かも...
   id_list: [1, 2, 3, 4]
   drive_mode: ['normal', 'normal', 'normal', 'normal']
   homing_offset_deg: [0.0, 0.0, 0.0, 0.0]
   return_delay_time_us: [0.0, 0.0, 0.0, 0.0]
   moving_threshold_deg_s: [0.0, 0.0, 0.0, 0.0]
   restore_configuration: [false, false, false, false]
   status_return_level: [2,2,2,2]    
   shutdown: [0b00000000, 0b00000000, 0b00000000, 0b00000000]
   led: [false, false, false, false]                    
   bus_watchbdog_ms: [0.0, 0.0, 0.0, 0.0]
   moving: []                 
   moving_status: []
   registered_instruction: []
   realtime_tick_us: []        
```
#### コマンドラインでの使用

```yaml  
$ ros2 topic echo --flow-style /dynamixel/debug #debug用, これがあると便利
status: # status型
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false]
   error: [false, false, false, false]
   ping: [true, true, true, true]
   mode: ['position', 'velocity', 'current', 'velocity']
current_ma: # pre_goal型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]
velocity_deg_s: # pre_goal型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]
position_deg: # pre_goal型
   present: [0.0, 0.0, 0.0, 0.0]
   goal: [0.0, 0.0, 0.0, 0.0]

$ ros2 topic echo --flow-style /dynamixel/state/limit # limit型
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

$ ros2 topic echo --flow-style /dynamixel/state/present # present型
id_list: [1,2,3,4]
pwm_pulse: [0.0, 0.0, 0.0, 0.0]
current_ma: [0.0, 0.0, 0.0, 0.0]
velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
position_deg: [0.0, 0.0, 0.0, 0.0]
vel_trajectory_deg_s: [0.0, 0.0, 0.0, 0.0]
pos_trajectory_deg: [0.0, 0.0, 0.0, 0.0]
input_voltage_v: []
temperature_degc: []
```

### write する情報

#### プログラムでの使用
プログラム上では以下のように使いたい
```cpp
dynamixel_handler::msg::DxlCommandsX cmd;
// id = 1,2,3,4 のサーボを torque_on.
cmd.common.command = "torque_on";
for (int i=1; i<=4; i++) cmd.common.id_list.push_back(i);
// id:1 のサーボを電流制御モードで50degに移動
cmd.current_base_position_control.id_list.push_back(1);
cmd.current_base_position_control.position_deg.push_back(50);
// id:2 のサーボのdゲインを50に設定
cmd.gain.id_list.push_back(2); 
cmd.gain.position_d_gain_pulse.push_back(50.0); 
// id:3 のサーボのgoal値を直接書き換え
cmd.goal.id_list.push_back(3);
cmd.goal.position_deg.push_back(100.0);//  goal値への書き込みはcontrol mode次第で有効
// より実践的には以下のように使う
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
`dynamixel_handler::msg::DxlCommandsX` の中身
```yaml
$ ros2 topic echo --flow-style /dynamixel/commands #このtopicはコマンドラインから送る想定ではない．
common: #common_cmd型
   command: "torque_on" #要素が1つの時は全idに適用
   id_list: [1,2,3,4]
pwm_control:
   id_list: [1,2,3,4]
   pwm_percent: [0.0, 0.0, 0.0, 0.0]
current_control: # cnt_x_cur型
   id_list: [1,2,3,4]
   current_ma: [0.0, 0.0, 0.0, 0.0]
velocity_control: # cnt_x_vel型
   id_list: [1,2,3,4]
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
position_control: # cnt_x_pos型
   id_list: [1,2,3,4]
   position_deg: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
extended_position_control: # cnt_x_e_pos型
   id_list: [1,2,3,4]
   position_deg: [0.0, 0.0, 0.0, 0.0]
   rotation: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
current_base_position_control: # cnt_x_c_pos型
   id_list: [1,2,3,4]
   current_ma: [0.0, 0.0, 0.0, 0.0]
   position_deg: [0.0, 0.0, 0.0, 0.0]
   rotation: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
status: # status型
   id_list: [1, 2, 3, 4]
   torque: [true, false, false, false] # torque_onコマンド, torque_offコマンドと同等
   error: [false, false, false, false] # clear_errorコマンドと同等
   ping: [true, true, true, true] # add_id コマンド, remove_id コマンドと同等
   mode: ['position', 'velocity', 'current', 'velocity'] # 各control系のコマンドと同等
goal: #goal型
   id_list: [1,2,3,4]
   pwm_pulse: [0.0, 0.0, 0.0, 0.0]
   current_ma: [0.0, 0.0, 0.0, 0.0]
   velocity_deg_s: [0.0, 0.0, 0.0, 0.0]
   profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
   profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
   position_deg: [0.0, 0.0, 0.0, 0.0]
gain: #gain型
   id_list: [1,2,3,4]
   velocity_i_gain_pulse: [0, 0, 0, 0]
   velocity_p_gain_pulse: [0, 0, 0, 0]
   position_d_gain_pulse: [0, 0, 0, 0]
   position_i_gain_pulse: [0, 0, 0, 0]
   position_p_gain_pulse: [0, 0, 0, 0]
   feedforward_2nd_gain_pulse: [0, 0, 0, 0]
   feedforward_1st_gain_pulse: [0, 0, 0, 0]
limit: #limit型
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
   id_list: [1, 2, 3, 4]
   drive_mode: ['normal', 'normal', 'normal', 'normal']
   homing_offset_deg: [0.0, 0.0, 0.0, 0.0]
   return_delay_time_us: [0.0, 0.0, 0.0, 0.0]
   moving_threshold_deg_s: [0.0, 0.0, 0.0, 0.0]
   restore_configuration: [false, false, false, false]
   status_return_level: [2,2,2,2]    
   shutdown: [0b00000000, 0b00000000, 0b00000000, 0b00000000]
   led: [false, false, false, false]                    
   bus_watchbdog_ms: [0.0, 0.0, 0.0, 0.0]
   moving: []                 
   moving_status: []
   registered_instruction: []
   realtime_tick_us: []          
```

#### コマンドラインでの使用
```yaml
ros2 topic echo --flow-style /dynamixel/command/common
command: "torque_on"
id_list: [1,2,3,4]

ros2 topic echo --flow-style /dynamixel/command/current_base_position_control
id_list: [1,2,3,4]
current_ma: [0.0, 0.0, 0.0, 0.0]
position_deg: [0.0, 0.0, 0.0, 0.0]
rotation: [0.0, 0.0, 0.0, 0.0]
profile_vel_deg_s: [0.0, 0.0, 0.0, 0.0]
profile_acc_deg_ss: [0.0, 0.0, 0.0, 0.0]
```

## external port に関して
こいつだけ, XH540シリーズだけで使える機能なので，独立させる．
これ難しいんだよな，ID × Port_num に対して，機能を設定したいんだけど...
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
