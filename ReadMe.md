# DynamixelHandler-ros2

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するための ros pkg `dynamixel_handler`を提供するリポジトリ.  

Dynamixelとやり取りを行うライブラリは[別のリポジトリ](https://github.com/SHINOBI-organization/lib_dynamixel)として管理しており，git submoduleの機能を使って取り込んでいる．

note: ROS2のみ対応

## features of this package
 - Dynamixelというサーボを動かすことに特化した最小単位のPkg
   - このPkgの dynamixel_handler node と ロボットの制御を行う別の node を組み合わせて使う
   - ほかのSDKと異なりコードを直接編集する必要なし
   - ほかのSDKと異なりコントロールテーブルのaddressや分解能などは意識する必要なし
   - 動かしたいDynamixelが「Jointとして使われているのか」「Wheelとして使われているのか」などの事前情報は不要
 - ROS topic の pub/sub のみでDynamixelを制御可能
   - 現在値やハードウェアエラーなどの情報を `dynammixel/states` topic として周期的にpub
      - present value デフォルト: 電流/速度/位置を50Hz
      - hardware error デフォルト: 約2Hz
      - その他 limit, gain, goal value など．
   - subした `dynamixel/commands/x` topic の内容に合わせて制御モードを自動で変更
   - sub/pubされる情報はパルス値ではなく物理量
      - 0 ~ 4095 の整数値ではなく，-180.0 ~ 180.0 の浮動小数値など. 
   - ユーザは純粋にサーボの目標状態を指令するだけでサーボを制御でき，また，同様にサーボの現在状態を受け取れる
 - 比較的高速なRead/Writeを実現 (12サーボに電流/速度/位置を Read/Write しても150Hzくらいは出せる)
   - 複数のアドレスを一括で読み書き & 複数のサーボを同時に読み書き(SyncRead/SyncWrite) によって Serial通信の回数を抑える
   - Fast Sync Read インストラクションを活用して Read を高速化
   - Raed/Write は ROS node の周期と同期しているので，topicのcallbackに左右されない．
   - ※ 適切な`LATENCY_TIMER`(2 ~ 4msくらい)と`baudrate`(1000000 ~ 推奨)の設定が必要
 - 開発における面倒を減らす機能
    - 初期化時に連結したDynamixelを自動で認識
    - 初期化時にエラーを自動でクリア (Optional)
    - 初期化時にトルクを自動でON (Optional)
    - node を kill したタイミングで動作を停止 
    - node を kill したタイミングでトルクをOFF (Optional)
    - エラークリア時の回転数消失問題を homing offset により自動補正
    - baudrateを一括で変更可能 (別nodeとして独立) 
 - ros param から各種 log 表示の制御が可能
   - Read/Write にかかる平均時間とSerial通信の成功率
   - Read/Write されるパルス値
   - Readに失敗したID
   - etc...

***************************

## how to install

### パッケージをgit clone
```bash
cd ~/ros2_ws/src
# sshの場合
git clone --recursive git@github.com:SHINOBI-organization/DynamixelHandler-ros2.git dynamixel_handler
# httpsの場合
git clone --recursive https://github.com/SHINOBI-organization/DynamixelHandler-ros2.git dynamixel_handler
```

### ビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to dynamixel_handler
source ~/.bashrc # 初回 build 時のみ
```

***************************

## how to use

### 1. Dynamixelの接続
- DynaimixelをディジーチェーンにしてU2D2経由でUSB接続されていること．
- idに重複がないように事前にDynamixel Wizardなどを用いて設定すること.
- baudrateが全て統一されていること．

> [!TIP]
> baudrateを一括で変更するための [dynamixel_unify_baudrate node](#Baudrateの一括変更) も用意してある

### 2. dynamixel_handler nodeの起動

#### 2-1. 自分の環境とconfigの設定を合わせる
config/config_dynamixel_handler.ymlの該当部分を編集し，保存．
以下はbaudrate: 1000000 かつ device name: /dec/ttyUSB0の場合
```yml
# config/config_dynamixel_handler.launch
/**:
    ros__parameters:
        # 通信機器の設定
        device_name: /dev/ttyUSB0 # 通信するデバイス名
        baudrate: 1000000 # 通信速度
```

通信環境の設定については[Parameters](#parameters)の章の[通信関係の設定](#通信関係の設定)を参照．

#### 2-2. ターミナルから実行
```bash
ros2 launch dynamixel_handler dynamixel_handler_launch.xml
```
出力例
```bash
# ... 略 ...
[dynamixel_handler_node-1] DynamixelHandler(): Initializing DynamixelHandler ..... 
[dynamixel_handler_node-1] Succeeded to open the port : /dev/ttyUSB0!
[dynamixel_handler_node-1] Succeeded to change the latency timer : 4!
[dynamixel_handler_node-1] Succeeded to change the baudrate : 2000000!
[dynamixel_handler_node-1] DynamixelHandler(): 
[dynamixel_handler_node-1] Expected number of Dynamixel is not set. Free number of Dynamixel is allowed
[dynamixel_handler_node-1] DynamixelHandler():  Auto scanning Dynamixel (id range [0] to [30]) ...
[dynamixel_handler_node-1] ScanDynamixels(): Scanning: 0 
[dynamixel_handler_node-1] addDynamixel():  * X series servo id [1] is found 
[dynamixel_handler_node-1] TorqueOn(): ID [1] is enabled torque 
[dynamixel_handler_node-1] ScanDynamixels(): Scanning: 5 
[dynamixel_handler_node-1] addDynamixel():  * X series servo id [6] is found 
[dynamixel_handler_node-1] TorqueOn(): ID [6] is enabled torque 
[dynamixel_handler_node-1] addDynamixel():  * X series servo id [7] is found 
[dynamixel_handler_node-1] TorqueOn(): ID [7] is enabled torque 
[dynamixel_handler_node-1] ScanDynamixels(): Scanning: 30
[dynamixel_handler_node-1] DynamixelHandler():   ... Finish scanning Dynamixel 
[dynamixel_handler_node-1] DynamixelHandler(): ..... DynamixelHandler is initialized 
[dynamixel_handler_node-1] MainLoop(): Loop [0]: write=0.00ms read=11.72ms(p/f=100%/100%) 
[dynamixel_handler_node-1] MainLoop(): Loop [300]: write=0.01ms read=5.55ms(p/f=100%/100%) 
[dynamixel_handler_node-1] MainLoop(): Loop [600]: write=0.01ms read=5.47ms(p/f=100%/100%) 
[dynamixel_handler_node-1] MainLoop(): Loop [900]: write=0.01ms read=5.30ms(p/f=100%/100%)
```

連結したDynamixelが自動で探索され，見つかったDynamixelの初期設定が行われる．
> [!TIP]
> 通信状態によっては連結しているのに見つからない場合もある.
> baudrateの確認，laytency timer の確認が有効である．  
> (laytency timer の確認は[LatencyTimer](#latencytimer)を参照．)  
> また，[Parameters](#parameters) の`dyn_comm/retry_num` を大きくすることでも改善する可能性がある．

初期化時の動作設定については[Parameters](#parameters)の章の[初期化時の動作設定](#初期化時の動作設定)を参照

### 3. Dynamixelを制御

コマンドラインから指令する用の topic `/dyanmixel/command/...` と プログラムから指令する用の topic `/dynamixel/commands/x` が用意されている．  
以下ではコマンドラインから指令を送る場合の例を示す．
プログラムから指令を送る場合は example を参照されたい．(あとでまとめる)

また，topic の詳細については [Topic](#topic) の章を参照．

#### 例：ID:5のDynamixel Xシリーズ のサーボを位置制御モードで角度を90degにする場合

`/dynamixel/command/x/position_control` topicにIDと角度を設定してpublish．
```bash
ros2 topic pub /dynamixel/command/x/position \
 dynamixel_handler/msg/DynamixelControlXPosition \
 "{id_list: [5], position_deg: [90], profile_vel_deg_s: [], profile_acc_deg_ss: []}" -1
```
> [!note]
> ID:5のDynamixelの制御モードは自動的に位置制御に変換される．

もしトルクが入っていなかった場合，`/dynamixel/command/x/common` topicに "torque_on" コマンドとIDを設定してpublish
```bash
ros2 topic pub /dynamixel/command/x/common \
 dynamixel_handler/msg/DynamixelCommonCmd \
 "{command: 'torque_on', id_list: [5]}"
```
ただし，デフォルトでは初期化時に自動でトルクONになっているため不要のはず．
`/dynamixel/command/x/common` topicで使えるコマンドについては，[Topic](#topic) の章を参照．

### 4. Dynamixelの情報を取得

コマンドラインから確認する用に`/dynamixel/state/...` topic 等と `/dynamixel/debug` topic として一定周期で raed & pub され続けている．
さらに，それらのすべてのトピックをまとめた topic `/dynamxiel/states` がプログラム用として用意されている．
以下ではコマンドラインから確認する例を示す．

topic の詳細については [Topic](#topic) の章を参照．
また，read周期については[Parameters](#parameters)の章の[実行時の動作設定](#実行時の動作設定)を参照．


#### 例: ID:5とID:6のモータが接続している場合の現在値の確認

```bash
ros2 topic echo --flow-style /dynamixel/state/present # status, goal, gain, limit, error... など． 
```

出力例
```yml
---
id_list: [5, 6] # 認識されているサーボのID
pwm_percent: [] # 現在のPWM値
current_ma: [0.0, -2.69] # 現在の電流値
velocity_deg_s: [0.0, 0.0] # 現在の各速度
position_deg: [89.91210937499999, -0.2636718750000023] # 現在の角度
vel_trajectory_deg_s: [] # 目標速度 みたいなもの
pos_trajectory_deg: [] # 目標角度 みたいなもの
temperature_degc: [] # 現在の温度
input_voltage_v: [] # 現在の入力電圧
---
```
read & pub される情報の選択については[Parameters](#parameters)の章の[実行時の動作設定](#実行時の動作設定)を参照．

#### 例: ID:1, 6, 7, 8, 9のモータが接続している場合のデバック用情報の確認

```bash
ros2 topic echo --flow-style /dynamixel/debug
```

出力例
```yml
---
status: # /dynamixel/state/status と同じ
  id_list: [1, 6, 7, 8, 9] # 認識されているサーボのID
  torque: [true, true, true, true, true] # トルクがONかOFFか
  error: [false, false, false, false, false] # エラーが発生しているか
  ping: [true, true, true, true, true] # pingが通っているか
  mode: [velocity, cur_position, cur_position, cur_position, cur_position] # 制御モード
current_ma: # 現在の電流値と目標電流値
  present: [0.0, 3.0, 2.0, 9.0, 4.0] 
  goal: [0.0, 900.0, 910.0, 910.0, 910.0]
velocity_deg_s: # 現在の速度と目標速度
  present: [0.0, 0.0, 0.0, 1.374, 0.0]
  goal: [0.0, 439.68, 439.68, 439.68, 439.68]
position_deg: # 現在の角度と目標角度
  present: [1007.5781, -24.7852, 44.1211, -33.8379, -87.8906]
  goal: [1007.666, -24.7852, 44.1211, -33.8379, -87.8906]
---
```

> [!NOTE]
> dynamixelからのread方式は Sync Read であり，すべてのIDから一斉にreadするようになっている．
> ただし，ros param `use/fast_read` が `true` の場合は  Fast Sync Read が用いられる．
> 
> また，上記の出力例にあるように複数の情報を読み込んでいるが，複数情報を一括でreadするか，分割でreadするかは，
> ros param `use/split_read` によって変更できる．
> 
> 分割でreadする場合は，読み込む情報の数分だけreadに時間がかかるので注意．
> 
> read方式については後述の[速度に関してメモ](#速度に関してメモ)を参照.

***************************

## Topic

さらなる詳細は[メッセージの定義](./msg/ReadMe.md)を参照

### Published from dynamixel_handler 

サーボの状態を監視するためのtopic.Xシリーズ，Pシリーズ共通．

 - `/dynamixel/states` (`DxlStates` type) : 下記の `/dynamixel/state/...` 系トピックすべてをひとまとめにしたtopic.プログラムでの使用を想定
 - `/dynamixel/state/status` (`DynamixelStatus` type) : サーボの状態(トルク・エラー・ping・制御モード)を確認するためのtopic
 - `/dynamixel/state/goal` (`DynamixelGoal` type) : サーボの目標値を確認するためのtopic
 - `/dynamixel/state/gain` (`DynamixelGain` type) : サーボの制御ゲインを確認するためのtopic
 - `/dynamixel/state/limit` (`DynamixelLimit` type) : サーボの制限値を確認するためのtopic
 - `/dynamixel/state/error` (`DynamixelError` type) : サーボのハードウェアエラー情報を確認するためのtopic
 - `/dynamixel/debug` (`DynamixelDebug` type) : コマンドラインからデバックする用のtopic

### Subscribed by dynamixel_handler 

Xシリーズのサーボへの入力を行うためのtopic.
> [!NOTE]
> 同様にPシリーズ用のトピックも用意されている．[メッセージの定義](./msg/ReadMe.md)参照

  - `/dynamixel/commands/x` (`DxlCommandsX` type) : 下記の `/dynamixel/command/...` 系トピックすべてをひとまとめにしたtopic. プログラムでの使用を想定
  - `/dynamixel/command/common` (`DynamixelCommonCmd` type) : dynamixelの起動や停止，エラー解除コマンドなどを送るためのtopic, 指定できるコマンドは[Command list](#command-list)参照       
  - `/dynamixel/command/x/pwm_control` (`DynamixelControlXCurrent` type) : Xシリーズを電流制御モード用
  - `/dynamixel/command/x/current_control` (`DynamixelControlXCurrent` type) : Xシリーズを電流制御モード用
  - `/dynamixel/command/x/velocity_control` (`DynamixelControlXVelocity` type) : Xシリーズを速度制御モード用
  - `/dynamixel/command/x/position_control` (`DynamixelControlXPosition` type) : Xシリーズを位置制御モード用
  - `/dynamixel/command/x/extended_position_control` (`DynamixelControlXExtendedPosition` type) : Xシリーズを拡張位置制御モード用
  - `/dynamixel/command/x/current_base_position_control ` (`DynamixelControlXCurrentPosition` type) : Xシリーズを電流制限付き位置制御モード用
  - `/dynamixel/command/status` (`DynamixelStatus` type) : サーボの状態(トルク・エラー・ping・制御モード)を設定するためのtopic (使わなくても, `/dynamixel/command/common` と `/dyanmixel/command/x/~_control`で設定できる)
  - `/dynamixel/command/goal` (`DynamixelGoal` type) : 各種の目標値を設定するためのtopic
  - `/dynamixel/command/gain` (`DynamixelGain` type) : 制御ゲインを設定するためのtopic
  - `/dynamixel/command/limit` (`DynamixelLimit` type) : 各種の制限値をを設定するためのtopic

#### Command list

`/dynamixel/command/common` topic `command` fieldに指定できる文字列.
`DynamixelCommonCmd`型の定義内で[定数として定義](./msg#dynamixelcommoncmd-type)されている．

- 高レベルコマンド：ユーザの利用を想定
  - `torque_on` / `TON`  : 安全にトルクをenableにする．目標姿勢を現在姿勢へ一致させ，速度を0にする．
  - `torque_off` / `TOFF`: トルクをdisableにする．
  - `clear_error` / `CE` : ハードウェアエラー(ex. overload)をrebootによって解除する．回転数の情報が喪失することによって現在角が不連続に変動する問題を解消するために，homing offset用いて自動で補正する．
  - `remove_id` / `RMID` : 指定したIDのサーボを認識リストから削除する．
  - `add_id` / `ADID`    : 指定したIDのサーボを認識リストに追加する．

- 低レベルコマンド：開発者向け
  - `reset_offset` : homing offset アドレスに 0 を書き込む．
  - `enable`  : torque enable アドレスに true を書き込む．
  - `disable` : torque enable アドレスに false を書き込む．
  - `reboot`  : reboot インストラクションを送る


***************************

## Parameters

### 通信関係の設定
```yml
# 通信機器の設定
  device_name: /dev/ttyUSB0 # 通信するデバイス名
  baudrate: 1000000 # 通信速度
  latency_timer: 4 # 通信のインターバル
# 通信の設定
  dyn_comm/retry_num: 10 # 通信失敗時のリトライ回数
  dyn_comm/inerval_msec: 5 # 通信失敗時のインターバル時間
  dyn_comm/verbose: false # 通信失敗時の詳細をエラーとして出すか
```
### 初期化時の挙動設定
```yml
# サーボの初期設定
  init/expected_servo_num: 0 # 期待するサーボの数，0ならいくつでもOK
  init/auto_search_min_id: 0 # 探索するサーボのIDの最小値
  init/auto_search_max_id: 20 # 探索するサーボのIDの最大値
  init/auto_search_retry_times: 10 # 探索のリトライ回数
  init/hardware_error_auto_clean: true # 初期化時に Hardware error を自動でクリアするかどうか
  init/torque_auto_enable: true # 初期化時に Torque を自動でONにするかどうか
  term/torque_auto_disable: true # 終了時に Torque を自動でOFFにするかどうか
  middle/no_response_id_auto_remove_count: 0 # 通信が途切れた場合に何カウントで自動で削除するかどうか
# デフォルト値の設定
  default/profile_acc: 600.0 # deg/s^2
  default/profile_vel: 100.0 # deg/s
```
`init/expected_servo_num` が `0`の時は，1つ以上servoが見つかるまで `init/auto_search_retry_times` の回数分スキャンを繰り返す．  
`init/expected_servo_num` が `0` でない場合は，その数だけservoが見つかるまで`init/auto_search_retry_times` の回数分スキャンを繰り返す．  
`init/auto_search_retry_times`の回数分のスキャンが失敗した場合，初期化失敗でノードは落ちる．

`default/profile_acc`と`default/profile_vel`は位置制御時の最大加速度と最大速度を決める．
この値が大きければキビキビとした動作になり，小さければ滑らかな動作になる．

### 実行時の動作設定
```yml
# ループの設定
  loop_rate: 100 # メインループの周期
  verbose_ratio: 300 # メインループの処理時間，通信の成功率を出力, ex 100なら100回に1回出力
  pub_ratio/present: # present_XXXを読み取り，/dynamixel/state/present トピックをpublish する割合
      pwm:                 0 # この回数に一回present_pwmを読み取る, 0=初回のみ
      current:             2 # この回数に一回present_currentを読み取る, 0=初回のみ
      velocity:            2 # この回数に一回present_velocityを読み取る, 0=初回のみ
      position:            2 # この回数に一回present_positionを読み取る, 0=初回のみ
      velocity_trajectory: 0 # この回数に一回velocity_trajectoryを読み取る, 0=初回のみ
      position_trajectory: 0 # この回数に一回position_trajectoryを読み取る, 0=初回のみ
      input_voltage:      29 # この回数に一回present_input_voltageを読み取る, 0=初回のみ
      temperature:        29 # この回数に一回present_temperatureを読み取る, 0=初回のみ
  pub_ratio/status: 47 # この回数に一回 Status を読み取り，/dynamixel/state/status トピックをpublish する, 0=初回のみ
  pub_ratio/goal: 11  # この回数に一回 Goal を読み取り，/dynamixel/state/goal トピックをpublish する, 0=初回のみ
  pub_ratio/gain: 101 # この回数に一回 Gain を読み取り，/dynamixel/state/gain トピックをpublish する, 0=初回のみ
  pub_ratio/limit: 307 # この回数に一回 Limit を読み取り，/dynamixel/state/limit トピックをpublish する, 0=初回のみ
  pub_ratio/error: 53 # この回数に一回 Hardware error を読み取り，/dynamixel/state/error トピックをpublish する, 0=初回のみ
# Read/Write方式
  use/fast_read: true # Fast Sync Read を使うかどうか
  use/split_read: false # 複数の情報を分割して読み取るかどうか
  use/split_write: true # 複数の情報を分割して書き込むかどうか
```
各情報 (status, present, goal, gain, limit, error) は topic 名と対応．
read と publish 周期は `loop_rate` を `pub_ratio/...` で割った値となる．
例: `loop_rate` = 100, `pub_ratio/status` = 47 の時 100/47 ≃ 2Hz．  

present の情報のみ各アドレス(pwm, current, velocity, ... , temperature)それぞれの読み取り割合を設定できる．
`~/present` トピックのpublish周期は `loop_rate` を `pub_ratio/present/...` の最小値で割った値となる．
例: `loop_rate` = 100, `pub_ratio/present/current` = 2 の時 100/2 = 50Hz．

### log出力関係
```yml
# デバッグ用
  max_log_width: 6 # 以下のlog出力で，サーボ何個ごとに改行を入れるか
  verbose/callback: true # コールバック関数の呼び出しを出力
  verbose/write_goal: false # 書き込みするgoalデータのpulse値を出力
  verbose/write_gain: false # 書き込みするgainデータのpulse値を出力
  verbose/write_limit: false # 書き込みするlimitデータのpulse値を出力
  verbose/read_status/raw: false # 読み込んだstatusデータのpulse値を出力
  verbose/read_status/err: false # statusデータの読み込みエラーを出力
  verbose/read_present/raw: false # 読み込んだpresentデータのpulse値を出力
  verbose/read_present/err: false # presentデータの読み込みエラーを出力
  verbose/read_goal/raw: false # 読み込んだgoalデータのpulse値を出力
  verbose/read_goal/err: false # goalデータの読み込みエラーを出力
  verbose/read_gain/raw: false # 読み込んだgainデータのpulse値を出力
  verbose/read_gain/err: false # gainデータの読み込みエラーを出力
  verbose/read_limit/raw: false # 読み込んだlimitデータのpulse値を出力
  verbose/read_limit/err: false # limitデータの読み込みエラーを出力
  verbose/read_hardware_error: true # 検出したHardware errorを出力
```

***************************

## Baudrateの一括変更

config/config_dynamixel_unify_baudrate.ymlの以下の部分を編集し，保存
``` yml
# config/config_dynamixel_unify_baudrate.yml
/**:
    ros__parameters:
        # 通信機器の設定
        device_name: /dev/ttyUSB0 # 通信するデバイス名
        target_baudrate: 1000000 # 統一したい通信速度
```
ターミナルを開いて次を実行
```bash
ros2 launch dynamixel_handler dynamixel_unify_baudrate_launch.xml
```
全てのdynamixelのbaudrateを`TARGET_BAUDRATE`に設定してくれる．変更が終わると自動でnodeは終了する．

***************************

## LatencyTimer

シリアル通信にはパケットの送受信の間にlatency timer分のインターバルが挟まる．
(USBデバイスのデフォルトは16msのようであり，高速な通信の妨げとなることが多い)
安定した通信のためには，使用するUBSデバイスの latency timer とros paramの `laytency_timer` を一致させる必要がある．

ros paramの変更には，config/config_dynamixel_handler.ymlの以下の部分を編集して保存する．
```yml
# config/config_dynamixel_handler.yml
latency_timer: 4 # 通信のインターバル
```

使用するUSBデバイスのlatency timerはコマンドラインから次のコマンドを実行することで変更できる．
基本的に1度だけ実行すればよい．
```bash
echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
rm 99-dynamixelsdk-usb.rules
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer # ttyUSB0の部分は環境に合わせて変更すること
```

一時的であれば以下のようにしてもよい．
ttyUSB0 の部分は自分の環境に合わせて編集すること．
```bash
echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

***************************

## Control Table との対応

以下説明はXシリーズの場合．Pシリーズの場合は適宜読み替えること．

### 状態 (status)
 - torque_enable  : 接続時に自動でトルクONされる. `/dynamixel/command/common`の`command`=`'torque_on'` or `'enable'`で1,`command`=`'torque_off'` or `'disable'`で0に設定される．  
 - operating_mode : 対応するtopicのsubで自動で設定される． 
 - (ping)         : Control table ではないが，statusとして扱っている．pingが通るかどうか．
 - (error)        : Control table ではないが，statusとして扱っている．何らかのエラーを持っているかどうか．

### 目標値 (goal)
 - goal_pwm             : 目標PWM値, PWM制御モードでのみ有効
 - goal_current         : 目標電流値, 電流制御モードと電流制御付き位置制御でのみ有効
 - goal_velocity        : 目標速度, 速度制御モードでのみ有効
 - goal_position        : 目標角度, 位置制御モードと拡張位置制御モード，電流制御付き位置制御で有効
 - profile_acceleration : 最大加速度, 速度制御・位置制御・拡張位置制御・電流制御付き位置制御モードで有効
 - profile_velocity     : 目標速度値, 位置制御モードと拡張位置制御モード，電流制御付き位置制御で有効 

`/dynamixel/commands/x` or 対応する`/dynamixel/command/x/~_control`系トピック or `/dynamixel/command/goal`をsubすると設定され，`loop_rate`の周期で書き込まれる．
また，`/dynamixel/states` or `/dynamixel/state/goal` として `loop_rate`/`pub_ratio/goal` の周期で読みだされ，publishされる．

### 現在値 (present)
 - present_pwm          : 現在のPWM値
 - present_current      : 現在の電流値
 - present_velocity     : 現在の速度
 - present_position     : 現在の角度
 - velocity_trajectory  : 目標速度のようなもの
 - position_trajectory  : 目標角度のようなもの
 - present_input_voltage: 現在の入力電圧
 - present_temperature  : 現在の温度

書き込みは不可．
`/dynamixel/states` or `/dynamixel/state/present` として `loop_rate`の内`pub_ratio/present/...`の最小値の割合で読みだされ，publishされる．

### 制限 (limit)
 - temperature_limit  
 - max_voltage_limit     
 - min_voltage_limit     
 - pwm_limit   
 - current_limit   
 - acceleration_limit   
 - velocity_limit      
 - max_position_limit    
 - min_position_limit
  
`/dynamixel/command/limit`をsubすると設定され，`loop_rate`の周期で書き込まれる．
また，`/dynamixel/states` or `/dynamixel/state/limit` として `loop_rate`/`pub_ratio/limit` の周期で読みだされ，publishされる．

### ゲイン (gain)
 - velocity_i_gain       
 - velocity_p_gain       
 - position_d_gain       
 - position_i_gain       
 - position_p_gain       
 - feedforward_acc_gain  
 - feedforward_vel_gain  
  
`/dynamixel/command/gain`をsubすると設定され，`loop_rate`の周期で書き込まれる．
また，`/dynamixel/states` or `/dynamixel/state/gain` として `loop_rate`/`pub_ratio/gain` の周期で読みだされ，publishされる．

note: 制御モードによってデフォルト値が異なり，なんとモードを変えると勝手に書き換えられてしまう．制御モードをまたぐ場合の処理については検討中．

### エラー (error)
 - hardware_error_status  : サーボのハードウェアエラー情報

`/dynamixel/state/error`として`loop_rate`のうち，`pub_ratio/error`に一回の周期でpubされる. 
  
### External Ports
 - external_port_mode_{x}
 - external_port_data_{x}

X540シリーズのみ搭載の機能．topicから制御可能．未実装だがすぐに対応する．

### その他 (extra)
 - drive_mode             : not support yet
 - return_delay_time      : not support yet
 - homing_offset          : ユーザーは使用不可，初期化時に0に設定され，reboot時の角度補正に用いられる．
 - moving_threshold       : not support yet
 - startup_configuration  : not support, buckupがあるときPIDゲインやprofile系の値を自動で復元してくれるが，PIDのデフォルト値がモードによって異なる問題があるので使わない．
 - shutdown               : not support yet
 - status_return_level    : not support, 常に2を前提とする
 - bus_watchbdog          : node kill時にサーボを自動停止させる機能に用いられる．   
 - led                    : not support yet
 - registered_instruction : not support yet
 - realtime_tick          : not support yet
 - moving                 : not support yet
 - moving_status          : not support yet

> [!note] 
> (bus_watchdog の設定値が1以上の時) bus_watchdogの設定値 × 20ms 通信がないと自動で動作停止処理が実行される．homing_offset が設定されている状態でこの動作停止処理が走るとなぜか homing_offsetだけ回転する．

***************************

### 未実装機能
 - External Ports関連の実装
 - マルチスレッド化
 - extra topic のread/writeの実装
 - command topic を service にする
   - 1対1通信になってしまって，利点が少ないのでは？
 - write するタイミングの変更
   - 現在の方法：sub callback でストアしメインループで write
     - [＋] write回数が抑えられる．
       - 各IDへの command が別の topic に乗ってきても，node 側で 1/roop_late [sec] 分の command をまとめてくれる
     - [＋] write の周期が一定以下になり，read の圧迫や負荷の変動が起きづらい
     - [－] 一度 command をストアするので，topic の sub から 最大 1/roop_late [sec] の遅延が生じてしまう．
       - 8ms未満くらいは遅れるが，そもそものtopicの遅延の方が支配的?(topic遅延が6ms，callback->writeが遅延2ms)
   - もう一つの方法：sub callback で直接 write
     - [＋] callback後の遅延は生じない
     - [－] topic の pub の仕方によってはwrite回数が増えてしまう
       - 例えば，ID:5へ指令する command topic と ID:6が別のノードからpubされているとすると，callbackは2回呼ばれる．一度ストアしてからまとめてWrite方式だとwriteは1回だが，callbackで直接Write方式だとwriteも2回

***************************

## 速度に関してメモ

### Sync Read vs Fast Sync Read(`use/fast_read`パラメータ)
結論としては，読み込むデータとサーボの数が少ないならFastを使う方がよい．

Fast Sync Readは受信するパケットが全サーボ分1つながりになっている．
つまり1回の通信でやり取りするパケット数が少なくなるので，同時に読み書きするサーボが多くなると速度に違いが出てくる．
少なくとも10サーボくらいで顕著にFast Sync Readの方が早くなる．

Fast Sync Read側のデメリットとしては，直列しているサーボのどこかで断線等が起きた場合に弱いという点が挙げられる．
Fast Sync Readはパケットがつながっているため，1つでも返事をしないサーボがあるとパケットが全滅してしまう．
（これはlib_dynamixelのパケット処理の実装が悪いかもしれないが，知識不足ですぐに改善できなさそう．）
通常のSync Readはパケットが独立しているため，断線するより前のサーボからの返事は受け取ることができる．
断線や接続不良が危惧されるような状況では通信周期を犠牲にして，Sync Readを使わざるを得ないだろう．

### 複数アドレスの同時読み込み(`use/split_read`パラメータ)
後述の書き込みと異なり，こちらは分割ではなく同時にするのが良い．
すなわち`use/split_read`は`false`を推奨する．

複数のアドレスからデータを読み込みたいとき，分割して読み込む場合はシリアル通信の処理時間が，アドレス数分だけ長くなる．
100Hz以上で回そうと思うと，present_current, present_velocity, present_positionという基本の3つを取り出すだけでもきつい．
自分の環境では，前述の3つくらいの同時読み込みであれば，120-180Hzくらいでる．200Hzは場合によって出るか出ないかというところ．
分割読み込みでは60-80Hzくらいで頭打ちとなってしまった．
present系の8つのアドレスすべてから読み込んでも，同時読み込みなら100Hzくらいはでる．
分割読み込みだと30Hzも怪しい．

（上記は全て， 14サーボ直列，lib_dynamixel側のLATENCY_TIMER=2ms, デバイス側のlatency timer=2ms, baudrate=1M での結果）

### 複数アドレスの同時書き込み(`use/split_write`パラメータ)
書き込みに関しては，同時ではなく分割するのが良いだろう．
すなわち`use/split_write`は`true`を推奨する．

自分の環境では，`use/split_write`を`false`の状態で，12サーボに goal_current, goal_velocity, profile_acc, profile_vel, goal_position を同時にSync Writeしようとしたら，書き込みが失敗してうまく動かなかった． 
書き込むサーボが少なければ動く．
また，`use_split_write`を`true`にして，分割で書き込み，1度に書き込むアドレスを減らしても動く．
書き込みに関しては，分割して行っても処理時間はほぼ変わらない(1ms未満しか遅くならない)ので，基本は`true`としておくべき．


***************************
***************************


## Trouble Shooting

### 「dynamixel_handlerがない」もしくは，「メッセージがない」といったエラーが出る場合

ターミナルを立ち上げ直すか，以下を実行．
```bash
source ~/.bashrc
```

### ``cannot publish data``といったようなエラーが出た場合
デフォルトのDDSはFast-RTPSであるが，固有のバグを持っているらしく，実行時にエラーが発生する．
そのため，DDSをEclipse Cyclone DDSに変更しておく．

```bash
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

※ ~/.bashrcの下部に``export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp``を追記しておくことで，\
ターミナルの立ち上げ時に毎回コマンドを打たなくて済む．

### wslにusbをアタッチしようとして以下のエラーが出たとき
`
usbipd: error: WSL 'usbip' client not correctly installed. See https://github.com/dorssel/usbipd-win/wiki/WSL-support for the latest instructions.
`
なんかようわからんが，以下のコマンドをwsl内で実行すると解決する．
```bash
sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
```

## Launchファイルと設定（yaml）
### launch_dynamixel_unify_baudrate.py
```bash
ros2 launch dynamixel_handler launch_dynamixel_unify_baudrate.py
```
対応する``yaml``は``config/config_dynamixel_handler.yaml``

### launch_dynamixel_handler.py
```bash
ros2 launch dynamixel_handler launch_dynamixel_handler.py
```
対応する``yaml``は``config/config_dynamixel_unify_baudrate.yaml``

※ 一度ビルドしていれば，yamlファイルの変更に伴うビルドは不要

