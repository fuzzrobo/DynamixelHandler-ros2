# DynamixelHandler-ros2

Robotis社の[Dynamixel](https://e-shop.robotis.co.jp/list.php?c_id=89)をROSから制御するための ros pkg `dynamixel_handler`を提供するリポジトリ.  

Dynamixelとやり取りを行うライブラリは[別のリポジトリ](https://github.com/SHINOBI-organization/lib_dynamixel)として管理しており，git submoduleの機能を使って取り込んでいる．

note: ROS2のみ対応

## ビルドから確認までの流れ

### パッケージをgit clone
```
cd ~/ros2_ws/src
git clone git@github.com:SHINOBI-organization/DynamixelHandler-ros2.git
mv DynamixelHandler-ros2 dynamixel_handler
```
※レポジトリの名前とパッケージ名が一致していない（後に変更する？）

### submoduleによって，別途参照しているパッケージをダウンロード
```
cd ~/ros2_ws/src/dynamixel_handler
cd dynamixel_handler
git submodule init
git submodule update
```

### ビルド
```
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to dynamixel_handler
```

### 確認
１つ目の確認
```
ros2 run dynamixel_handler dynamixel_unify_baudrate_node
```
※「dynamixel_handlerがない」もしくは，「メッセージがない」といったエラーが出る場合，\
ターミナルを立ち上げ直すか，以下を実行．
```
source ~/.bashrc
```

２つ目の確認
```
ros2 run dynamixel_handler dynamixel_handler_node
```

``cannot publish data``といったようなエラーが出た場合，\
後述の「implementation DDSについて」を参照

## Launchファイルと設定（yaml）
### dynamixel_unify_baudrate_launch.py
```
ros2 launch dynamixel_handler dynamixel_unify_baudrate_launch.py
```
対応する``yaml``は``config/dynamixel_handler.yaml``

### dynamixel_handler_launch.py
```
ros2 launch dynamixel_handler dynamixel_handler_launch.py
```
対応する``yaml``は``config/dynamixel_unify_baudrate.yaml``

※ 一度ビルドしていれば，yamlファイルの変更に伴うビルドは不要

## 初期設定と注意事項

### latencyについて
`include\mylib_dynamixel\download\port_handler_linux.cpp` 内の `LATENCY_TIMER` と使用するUBSデバイスのlatency timerを一致させる必要がある．
このライブラリのデフォルトは`LATENCY_TIMER=16`で，USBデバイス側のデフォルトと等しいので通常は問題にならないが，高速化したいとき問題となる．

`include\download\port_handler_linux.cpp` 内の `LATENCY_TIMER`は，該当部分を書き換えてコンパイルでOK

使用するUSBデバイスのlatency timerは次のようにして変更する．
```
$ echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
$ sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
$ sudo udevadm control --reload-rules
$ sudo udevadm trigger --action=add
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

一時的であれば以下のようにしてもよい．
```
$ echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
$ cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

<!-- コピペ用

```
echo ACTION==\"add\", SUBSYSTEM==\"usb-serial\", DRIVER==\"ftdi_sio\", ATTR{latency_timer}=\"4\" > 99-dynamixelsdk-usb.rules
sudo cp ./99-dynamixelsdk-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --action=add
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

```
echo 4 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
``` -->

### implementation DDSについて
デフォルトのDDSはFast-RTPSであるが，固有のバグを持っているらしく，実行時にエラーが発生する．
そのため，DDSをEclipse Cyclone DDSに変更しておく．

```
$ sudo apt update
$ sudo apt install ros-humble-rmw-cyclonedds-cpp
$ export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
<!-- 
コピペ用
```
sudo apt update
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
``` -->

※ ~/.bashrcの下部に``export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp``を追記しておくことで，\
ターミナルの立ち上げ時に毎回コマンドを打たなくて済む．