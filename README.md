# pr
 pr用ROSパッケージ。足回りや自己位置推定、自律走行用プログラムが入っている。
 
# 依存パッケージ
ros-kinetic-amcl  
ros-kinetic-urg_node  
ros-kinetic-joy-node  
ros-kinetic-rosserial  
ros-kinetic-rosserial-arduino  
ros-kinetic-map-server

# 注意事項
 - pythonのスクリプトは、まず実行権限を与え必る要あり。 sudo chmod +x とか sudo chmod 777 を使う。  
 - start_localization1.launch,start_localization2.launchを立ち上げてjoystick操作で手動走行させる場合、map座標系から見てロボットを移動させたい方向にスティックを倒す。  
 
# ノードの説明
pr_arduino_driver.py  
pr_tf_listener.py  
pr_task_manager.py  
pr_virtual_omni.py  
  
pr_arduino_driver.py: arduinoへのモーター回転速度指令の伝達、ジャイロセンサを用いたカルマンフィルタの実行、エンコーダを用いたオドメトリの計算を行う。  

	pub,sub  
	[トピック名]（メッセージ型）：説明  
	
	pub  
	[raw_power] (pr/RawPower): int8の要素を３つもつ。３輪オムニの3つのモーターの回転速度指令。各要素の値は-100から100。この値は、モーターを最大回転速度の何%で回転させるかを表す。負の値ではモーターが逆転する。  
	sub
	[raw_encoder] (pr/RawEncoder): int16の要素を３つもつ。伊勢モードラに接続させたエンコーダからのデータが格納される。
	[cmd_vel] (geometry_msgs/Twist): ロボットの並進速度と角速度。
	[yaw_base2map] (std_msgs/Float32): map座標系から見たロボットのヨー角。単位はラジアン。-πからπの値。
	[gyro] (std_msgs/Float32): ジャイロセンサから得られた、odom座標系から見たロボットのヨー角(の推定値)。単位は°。任意の実数値をとる。  
  
pr_tf_listener.py: tf情報を読み取り、/mapから見た/base_linkのyaw角を取得、それをyaw_base2mapにパブリッシュする。  

	pub
	[yaw_base2map] (std_msgs/Float32): 前述した通り。
	sub
	[tf]: TransForm
	
pr_task_manager.py: joyトピックをサブスクライブし、joystickの操作コマンドを変換してcmd_velトピックにパブリッシュする。

	pub
	[cmd_vel] (geometry_msgs/Twist): joystickからのデータをもとに計算された、ロボットの並進速度と角速度。
	sub
	[joy][sensor_msgs/Joy]: joy_nodeから送られてくる、joystickの操作コマンド。
 
 pr_virtual_omni.py: ヴァーチャル３輪オムニ。操作系のデバッグに使ったりする。通常は使用しない。
 
 	pub
	[raw_encoder] (pr/RawEncoder): 省略。
	sub
	[raw_power] (pr/RawPower): 省略。

# launchファイル
controller.launch  
get_data.launch  
start_amcl_sim.launch  
start_localization.launch  
start_localization2.launch  
simulator.launch  
  
 - controller.launch: 自己位置推定無しで、joystickで手動走行させるときに使う。ロボット座標系から見てロボットを移動させたい方向にjoystickの左スティックを倒す。右スティックは回転用。左に倒すと反時計回りに回転し、右に倒すと時計回りに回転する。  
    立ち上げるノード:  
    pr_arduino_driver.py  
    pr_task_manager.py  
    serial_node.py(モーター制御用arduino mega)  
    ※注意：このlaunchファイルを立ち上げる際、state.yaml内のuse_pfとuse_kfの値を0にすること。  
  
- get_data.launch: rosbagでセンサデータを取って自己位置推定シミュレーションを行う時に使う。センサデータ(トピック:scan,raw_encoder,gyro)をパブリッシュするノードと手動走行用のノードを立ち上げる。  
     立ち上げるノード:  
     pr_arduino_driver.py  
     pr_task_manager.py  
     urg_node  
     joy_node  
     serial_node.py(モーター制御用arduino mega)  
     serial_node.py(ジャイロセンサ用arduino uno)  
     ※注意：このlaunchファイルを立ち上げる際、state.yaml内のuse_pfとuse_kfの値を0にすること。  
  
 - start_amcl_sim.launch: rosbagで取ったデータをもとに自己位置推定シミュレーションを行う時に使う。  
     立ち上げるノード:  
     amcl  
     pr_arduino_driver.py  
     map_server  
  
 - start_localization1.launch: joystickでの手動走行と自己位置推定を同時に行う時に使う。最初に1を立ち上げ、立ち上がったのを確認したら2を立ち上げる。  
     立ち上げるノード:  
     joy_node  
     amcl  
     pr_arduino_driver.py  
     urg_node  
     map_server  
  
 - start_localization2.launch: start_localization1.launchに続いて立ち上げる。  
     立ち上げるノード:  
     pr_tf_listener.py  
     pr_task_manager.py  
     serial_node.py(モーター駆動用arduino mega)  
     serial_node.py(ジャイロセンサ用arduino uno)  
  
 - simulator.launch: joystickで仮想3輪オムニを動かす時に使う。自己位置推定はしない。  
     立ち上げるノード:
     pr_arduino_driver.py  
     pr_virtual_omni.py  
     joy_node  
     pr_task_manager.py  
  
# yamlファイル
 - constant.yaml:プログラムで使う各種の定数を格納している。  
     
 - state.yaml: カルマンフィルタやamclによる自己位置推定結果を用いるかどうかや、走行パターン(自動・手動)の切り替えを行う。  
  
# pr_arduinoフォルダ
 - pr_arduino.ino  
    モーター制御用arduino megaに書き込むプログラム。  
    pr_arduino_driver.pyから送られてくる速度指令をサブスクライブし、伊勢モードラにduty比を送る + 伊勢モードラからエンコーダの値を読み取り、rosにパブリッシュ + モーターの回転速度のPD制御を行う。  
 - MPU6050_manual_read  
    ジャイロセンサ用arduino unoに書き込むプログラム。ジャイロセンサからの角速度を読み取って回転角度を計算し、gyroトピックにパブリッシュする。
