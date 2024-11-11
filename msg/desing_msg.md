# Custom msg design

## お気持ち
 - `\#inclde ~` する数を減らしたい -> 大きめの msg を作る．
 - コマンドラインから制御 or 確認するときに大きな msg はだるいので小さくしたい -> 入れ子構造で作る

## topic list 
想定するtopicのリストを示す．
```bash
### publish
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
/dynamixel/ex_port/read # ex_port型

### subscribe
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
dynamixel_handler::msg::DynamixelStates msg; //すべての状態が確認できる
// statusの確認 
for (size_t i = 0; i < msg.status.id_list.size(); i++) {
   auto id = msg.status.id_list[i];
   auto torque = msg.status.torque[i] ? "on" : "off";
   auto error = msg.status.error[i] ? "error" : "no error";
   auto ping = msg.status.ping[i] ? "response" : "no response";
   auto mode = msg.status.mode[i];
   printf("servo [%d], torque %s, has %s, ping is %s, mode is %s\n", id, torque, error, ping, mode); 
}
// 現在値の確認, present valueはすべての要素があるとは限らないので確認が必要
if ( msg.present.current_ma.empty()     ) return;
if ( msg.present.velocity_deg_s.empty() ) return;
if ( msg.present.position_deg.empty()   ) return;
for (size_t i=0; i < msg.present.id_list.size(); i++) {
   auto id = msg.present.id_list[i];
   auto current_ma = msg.present.current_ma[i];
   auto velocity_deg_s = msg.present.velocity_deg_s[i];
   auto position_deg = msg.present.position_deg[i];
   printf("servo [%d], current %f, velocity %f, position %f\n", id, current_ma, velocity_deg_s, position_deg)
}
```
`dynamixel_handler::msg::DynamixelStates` の中身
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
```
`dynamixel_handler::msg::DxlCommandsX` の中身
```yaml
$ ros2 topic pub /dynamixel/commands dynamixel_handler::msg::DxlCommandsX  #このtopicはコマンドラインから送る想定ではない．
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
ros2 topic pub /dynamixel/command/common dynamixel_handler::msg::DynamixelCommonCmd
command: "torque_on"
id_list: [1,2,3,4]
```

## external port に関して
こいつだけ, XH540シリーズだけで使える機能なので，独立させる．

### Read
```yaml
$ ros2 topic echo --flow-style /dynamixel/ex_port/read # あんまり一般的じゃないし，独立させようかな
stamp: 0000
id_list: [1, 2, 3, 4]
port_1:
   mode: ['read', 'read', 'read', 'read']
   value: [0, 0, 0, 0]
port_2:
   mode: ['write', 'read', 'read', 'read']
   value: [0, 0, 0, 0]
port_3:
   mode: ['write', 'read', 'read', 'read']
   value: [0, 0, 0, 0]
port_4:
   mode: []
   value: []
```
### Write
```yaml
$ ros2 topic pub /dynamixel/ex_port/write dynamixel_handler::msg::DynamixelExternalPort
id_list: [1, 2, 3, 4]
port_num: [1, 1, 1, 2]
port: 
   mode: ['read', 'read', 'read', 'read']
   value: [0, 0, 0, 0]
```

