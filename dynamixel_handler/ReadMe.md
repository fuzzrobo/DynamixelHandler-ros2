# 開発者向けReadMe

## プログラムの構成

Dynamixelとやり取りを行うライブラリは[別のリポジトリ](https://github.com/SHINOBI-organization/lib_dynamixel)として管理しており，git submoduleの機能を使って取り込んでいる．
現在の取り込み先は `src/internal/mylib_dynamixel` である．

`develop/faster_build` マージ後は，`dynamixel_unify_baudrate` が Python ノード実装になっており，
内部で pybind11 モジュール `_mylib_dynamixel` を利用する構成になっている．

このパッケージはROS2ノードとして動作し，srcディレクトリ以下に主要な実装が配置されています．各ファイルの役割は以下の通りです:

- **src/main.cpp**  
  ノードのエントリーポイント．
  `DynamixelHandler` の生成，executor への登録，spin を行う最小構成．

- **src/dynamixel_handler.cpp**  
  `DynamixelHandler` クラス本体の実装．
  初期化処理，パラメータ読込，publisher/subscriber生成，optional機能の起動，
  および `MainLoop()` の実処理（read/write，publish）を担当する．

- **src/dynamixel_handler.hpp**  
  本パッケージの中心となるクラスの宣言を行い，Dynamixelとの通信および各種データ（状態，コマンド，制御パラメータ）の管理を実装しています．

- **src/dynamixel_handler_ros_interface_sub.cpp**  
  ROSのsubscribeインターフェースを担当し，各種コマンドメッセージの受信と一時的なコマンド情報の保持についての処理を実装しています．

- **src/dynamixel_handler_ros_interface_pub.cpp**  
  ROSのpublishインターフェースを担当し，Dynamixelの状態やエラー情報，デバッグ情報などを各トピックに出力する処理を実装しています．

- **src/dynamixel_handler_ros_setup_for_program.cpp**  
  プログラム連携向けのROS interface初期化を担当し，
  `dynamixel/commands/{all,x,p,pro}` と `dynamixel/states` / `dynamixel/debug` の生成を行う．

- **src/dynamixel_handler_ros_setup_for_cli.cpp**  
  CLI運用向けのROS interface初期化を担当し，
  `dynamixel/command/...` 系・`dynamixel/state/...` 系など細分化トピックの生成を行う．

- **src/dynamixel_handler_dyn_interface_loop.cpp**  
  動作ループ内での一括読み書き処理（SyncRead，SyncWrite）を担当し，各サーボの状態やエラー情報の更新，コマンドの指令，状態の取得などを実装しています．

- **src/dynamixel_handler_dyn_interface_once.cpp**  
  動作ループ外での一括読み書き処理（Read，Write）を担当し，各サーボの状態やエラー情報の更新，コマンドの指令，状態の取得などを実装しています．

- **src/optional_function/**  
  optional機能の実装ディレクトリ．
  現在は `external_port.cpp/.hpp` と `imu_opencr.cpp/.hpp` を含む．

- **src/sub_node/dynamixel_unify_baudrate.py**  
  baudrate一括変更ノード本体（Python）．
  pybind11モジュール `_mylib_dynamixel` を介してシリアル通信を行う．

さらに，`src/internal/myUtils` 内のユーティリティ群は，ログ出力やフォーマッティング，イテレータの利便性向上など，本パッケージ全体で共通して利用される補助的な機能を提供している．
`src/internal/mylib_dynamixel` は通信ライブラリ本体（submodule）であり，`dynamixel_communicator` と `_mylib_dynamixel` のビルド元になっている．

***************************

## 残課題
 - `extra` の公開トピック仕様の整理
 - `ChangeOperatingMode()` の危険性整理と対策検討
   - これは async 化とは別件の既存課題．
   - `ChangeOperatingMode()` は operating mode の readback で成否を確定する前に，`goal_w_` / profile 系の値を書き戻している．
   - そのため，利用者が「変更後モードでは停止するつもり」の command を出しても，mode 変更が失敗して旧モードのまま残ると，旧モード向けの goal が効いて動き始める組合せがある．
   - ここで使う略記は以下とする．
     - `p0`: `goal_pwm == 0`
     - `c0`: `goal_current == 0`
     - `v0`: `goal_velocity == 0`
     - `x0`: `goal_position == present_position`
     - `p* / c* / v* / x*`: 上記の逆で，その mode では動き得る値
   - とくに危険なのは，旧モードが position 系 (`POSITION`, `EXTENDED_POSITION`, `CURRENT_BASE_POSITION`) で，変更後モードが non-position 系 (`PWM`, `CURRENT`, `VELOCITY`) の場合．
     - 例: `pre = POSITION`, `after = PWM`, `goal_pwm = 0`, ただし `goal_position != present_position`
     - 利用者の期待: PWM 0 なので停止
     - 実際: mode 変更が成功すれば停止するが，失敗して old mode = POSITION のままだと `goal_position` が効いて動く
   - 同型の危険例
     - `POSITION / EXTENDED_POSITION / CURRENT_BASE_POSITION -> CURRENT` で `goal_current = 0` かつ `goal_position != present_position`
     - `POSITION / EXTENDED_POSITION / CURRENT_BASE_POSITION -> VELOCITY` で `goal_velocity = 0` かつ `goal_position != present_position`
   - 重要なのは，この危険は「その周期に新しく goal を受けた場合」だけではないこと．`goal_w_` に以前から残っていた値でも起きる．つまり mode-only request でも発生し得る．
   - 一方で，`ChangeOperatingMode()` 単体で特に危険なのは旧モードが position 系のケースである．旧モードが `PWM/CURRENT/VELOCITY` のときは，この関数単体では旧モードの主 goal を再送しない．
   - ただし，その後の通常の `SyncWriteGoal()` 経路まで含めると，old mode が `PWM/CURRENT/VELOCITY` の組合せにも別種の危険は残り得るので，`ChangeOperatingMode()` 単体の問題と `MainLoop()` 全体の問題を分けて整理すること．
   - 対策案としては，少なくとも以下を比較検討する．
     - readback mismatch を「通信失敗」ではなく「unsupported / semantic failure」とみなして retry を打ち切る
     - mode 変更を試した周期は，当該 ID への `goal` 再送を抑止する
     - `ChangeOperatingMode()` 内での goal/profile 書き戻し順序自体を見直す
 - `CallbackCmd_Status()` の `series_[ID] == P` フォールバック整理
   - これも async 化とは別件の既存課題．
   - 現状は `CURRENT_BASE_POSITION` を受けたときに，`series_[ID] == P` なら `EXTENDED_POSITION` へフォールバックしている．
   - しかし operating mode の対応可否は series 単位ではなく model ごとの差が大きく，この実装は明らかに片手落ちである．
   - 現状の問題点
     - `P` series の 1 ケースだけ特別扱いしており，一貫した capability 判定になっていない
     - `X` series 内でも model により対応 mode が異なり得るのに見ていない
     - callback 側で暫定変換しているため，将来 `change_id` や `ping add + mode` などの同周期処理を見直すときに責務が曖昧になりやすい
   - 今後この論点を触るときは，少なくとも次の方針を比較する．
     - callback 側の `series` ベース fallback をやめ，model ベース capability table を持つ
     - callback 側では変換せず raw request のまま保持し，write + readback 結果で supported / unsupported を判定する
   - 少なくとも現状の `series_[ID] == P` だけの特殊処理は「暫定実装」と明記し続けること．
 - `ExternalPort::CallbackExternalPort()` の重複除去ロジック見直し
   - 現状の重複除去は `(id, port)` の組ではなく，`id` と `port` を別々の list に積んで `is_in(id, valid_id_list) && is_in(port, valid_port_list)` で判定している．
   - そのため，たとえば `(2,1)` と `(4,2)` を受理済みの同一 message 内で `(2,2)` が来ると，重複ではないのに重複扱いで捨てられる．
   - 本来は `(id, port)` pair 単位で dedupe すべきであり，現在の実装は external port を 2 port 以上使う構成で機能バグになり得る．
   - 自動 fixture では未検証なので，少なくとも手動 case を維持しつつ，将来的には pair 単位の回帰 test を追加すること．
 - write するタイミングの検討について
   - 現在の方法：sub callback でストアしメインループで write
     - [＋] write回数が抑えられる．
       - 各IDへの command が別の topic に乗ってきても，node 側で 1/loop_rate [sec] 分の command をまとめてくれる
     - [＋] write の周期が一定以下になり，read の圧迫や負荷の変動が起きづらい
     - [－] 一度 command をストアするので，topic の sub から 最大 1/loop_rate [sec] の遅延が生じてしまう．
       - 8ms未満くらいは遅れるが，そもそものtopicの遅延の方が支配的?(topic遅延が6ms，callback->writeが遅延2ms)
   - もう一つの方法：sub callback で直接 write
     - [＋] callback後の遅延は生じない
     - [－] topic の pub の仕方によってはwrite回数が増えてしまう
       - 例えば，ID:5へ指令する command topic と ID:6が別のノードからpubされているとすると，callbackは2回呼ばれる．一度ストアしてからまとめてWrite方式だとwriteは1回だが，callbackで直接Write方式だとwriteも2回

## 開発メモ

### Node終了時の停止について

- bus_watchdogで停止できそう？
  - RAM値なので簡単に書き込みできる
  - 初期化時の最初に書き込まないと，bus_watchdogがエラー中のサーボのgoal値を書き換えられない
  - Velocity・current制御モードの場合だけ停止するようにしていたが，全部停止するようにしてもいいかもしれない．
- bus_watchdogのバグあり．
  - XC330は停止しない？
  - XM540は停止はするけどhoming_offsetの値分動いてから停止する．なんで？？？ (Firmware update 46 to 48 で治った？治ってないやんけ！)


1. 通信断絶時の停止について
   - 速度・電流・PWM制御モードではデフォルトでbus_watchdogによって止まるようにしたい．
   - `term/servo_auto_stop`パラメータで制御する？
   - 取りあえずbus_watchdogでだいたいは大丈夫なはず．
     - XM540の位置制御系では，homing_offsetが設定時のバグが残っているが，速度・電流・PWM制御系では問題ない．
   - (例外) XC330はそもそも止まらない．
     - こいつのみ別途止める必要がある？ないか．
     - まあ，通信断絶
  
2. ノードの明示的な停止について
    - こいつも `term/servo_auto_stop` で制御する．
    - 速度・電流・PWM制御モードについてはbus_watchdogでクリアしている．
      - 念のため即時停止するように最も短いbus_watchdogを再設定するか？
    - 位置制御系については現在位置を指令しなおすしか手がないかな．

3. トルクの瞬間的なオンオフでは止まらなそう．
   - XC330は止まらずに最初の目標値目指して回り続ける (goal値はオンオフされた瞬間のものに更新されてる)
   - XM540は止まるときと，止まらずに加速するときがある．わけわからん．
  
### 遭遇したバグ

Fast Sync Read で 1...6 のサーボに read を送った状況で，ID:1モータが断線していたとする．  
当然，Fast Sync Read の return packet であるひとつながりのパケットは帰ってこないのでタイムアウト．   
その直後，1...6の各モータに対してPingを送るとなぜか**全て**のモータがタイムアウトする．  
「Fast Sync Read + 1つ目のIDの断線によりタイムアウト + 左記の直後に Ping」という状況だと，断線してないモータもタイムアウトしてしまうという問題が起きる．

 - Sync Read だとこの問題は起きない．
 - Fast Sync Read で指定するIDの先頭であるID:1 **以外** のモータを断線させても問題は起きない．
 - Fast Sync Read で指定するIDの順番を逆順にしたうえで，ID:1を断線させても問題は起きない．
 - Fast Sync Read で指定するIDの順番を逆順にしたうえで，ID:6を断線させると問題が起きる．

### Yシリーズ対応に向けて

- 方針としては何とか頑張ってX, Pシリーズと共通化する方向で行きたい．
- ただし，X, Pシリーズの方がYシリーズよりも機能が多いので，msgのfieldはYシリーズに合わせる形になるかも．

Commands 系: 個別に用意すればいいので問題なし?
  - goal: 順番は異なるが機能は同じ, profileのvelベースとtimeベースが分離してたりするが機能は同じ．
    - 結局Goal系はIndirect Addressで再構築しないと難しそう．
    - Profile_vel, Profile_accは, Profile_vel, Profile_acc, Profile_time, Profile_acc_timeの4つに分裂しているが，結局Drive_modeの設定でどのペアが使われるかが変わるので，以下のようにして他のシリーズと共用の形式にまとめられそう．
    ```
    float64[] profile_vel_deg_s  # こちらが埋まっている場合は drive_mode を velocity base　に
    float64[] profile_acc_deg_ss # こちらが埋まっている場合は drive_mode を velocity base　に
    float64[] profile_time_ms     # こちらが埋まっている場合は drive_mode を time base に + Yシリーズの場合はIndirect Address の方もvel->timeに書き換え
    float64[] profile_acc_time_ms # こちらが埋まっている場合は drive_mode を time base に + Yシリーズの場合はIndirect Address の方もvel->timeに書き換え
    ```
  - offset: PWM, Current, Velocityについて謎の項目が生えてる．
States 系: 1つのmsgに載せられるかは要検討
  - gain: 順番は異なるが同じ
  - present: 温度がモータとインバータで別れてる．他は同じ．項目を追加すれば対応可能．
  - limit: 同上
  - error: 種類の増加はともかく，エラーデータ構造が異なっている．種類増加は単純にフィールド追加で対応，データ構造はYシリーズのみ読み込み方を変えることで対応できそう．

順番違い系はIndirectAddressで何とかするか...？


エラークリア用のパケットが増えてる

## optional_function 実装ルール（開発者向け）

`optional_function` は「Dynamixelの基本制御に依存しない拡張機能」を追加するための層として扱う．
既存の `ExternalPort` と `ImuOpenCR` は同じ流儀で実装しているので，以下を共通ルールとする．

### 追加時の実装手順

#### 1) 新規作成するファイル

```text
src/optional_function/<function_name>.hpp
src/optional_function/<function_name>.cpp
```

#### 2) 既存ファイルの編集ポイント

`src/dynamixel_handler.hpp`

```cpp
class <FunctionClass>;
std::unique_ptr<FunctionClass> <function_ptr>_;
```

`src/dynamixel_handler.cpp`

```cpp
#include "optional_function/<function_name>.hpp"

bool use_<function_name>;
get_parameter_or("option/<function_name>.use", use_<function_name>, false);
if (use_<function_name>) {
    <function_ptr>_ = std::make_unique<FunctionClass>(*this);
}

// MainLoop() 内
if (<function_ptr>_) <function_ptr>_->MainProcess();
```

`config/config_dynamixel_handler*.yaml`

```yaml
option/<function_name>:
    use: false
    pub_ratio: 10
    verbose/callback: false
    verbose/write: false
    verbose/read: {raw: false, err: false}
```

README (`../ReadMe.md` / `ReadMe.md`) も同時に更新する．

### パラメータ設計のルール

#### config 側（yaml）のルール

- 有効化フラグは `option/<function_name>.use` を必須とする．
- 周期制御は `pub_ratio`（または `pub_ratio/<kind>`）で統一する．
- verbose系は `verbose/callback`, `verbose/write`, `verbose/read.{raw,err}` を基本形とする．

#### cpp 側（読込実装）のルール

```cpp
parent_.get_parameter_or("option/<function_name>.pub_ratio", pub_ratio_, 10u);
parent_.get_parameter_or("option/<function_name>.verbose/callback", verbose_callback_, false);
parent_.get_parameter_or("option/<function_name>.verbose/write",    verbose_write_,    false);
parent_.get_parameter_or("option/<function_name>.verbose/read.raw", verbose_read_,     false);
parent_.get_parameter_or("option/<function_name>.verbose/read.err", verbose_read_err_, false);
```

- パラメータ読込は `get_parameter_or` を使い，デフォルト値をコード上で明示する．
- configキー名とcppキー名を必ず一致させる．

### 初期化と失敗時挙動

#### 初期化時

- optional機能の初期化失敗でノード全体を停止しない（`ROS_STOP`しない）．
- 失敗時は `WARN` を出し，当該機能のみ無効化して続行する．
- 依存先（例: firmware の model/address）が不一致な場合は，理由をログに残して無効化する．

### 実行時処理のルール

- `MainProcess()` は極力軽量にし，ループ周期を阻害しない．
- read成功時のみpublishし，通信失敗時は `verbose/read.err` 設定に従って警告を出す．
- callback内で待機が必要な場合は用途を明確化し，ログを出す（例: IMU校正の待機）．

### 命名と責務分離

- クラス名は機能単位で明確にする（例: `ExternalPort`, `ImuOpenCR`）．
- 機能固有のtopic/param名は `dynamixel/<function>` および `option/<function>` に寄せる．
- サーボ制御の共通ロジック（goal/present/gain/limit/error）に不要な依存を持ち込まない．

### 実装前後チェックリスト

#### ファイル構成

1. `optional_function/<function_name>.hpp/.cpp` を作成したか．
2. `dynamixel_handler.hpp` に前方宣言と `unique_ptr` を追加したか．
3. `dynamixel_handler.cpp` に生成条件と `MainLoop` 呼び出しを追加したか．

#### 挙動

1. 初期化失敗時に「機能のみ無効化」で続行できるか．
2. `verbose` 設定で read/write/callback のログ粒度を制御できるか．
3. read失敗時に通信エラーを握りつぶさず，必要時に警告できるか．

#### ドキュメント

1. `config` と README の記述が実装と一致しているか．
2. topic名・param名・デフォルト値がREADMEに反映されているか．
