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
 - start_localization.launch,start_localization2.launchを立ち上げてjoystick操作で手動走行させる場合、map座標系から見てロボットを移動させたい方向にスティックを倒す。  
 
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

# yamlファイル
constant.yaml  
state.yaml

# pr_arduinoフォルダ
 - pr_arduino.ino  
モーター制御用arduino megaに書き込むプログラム。  
pr_arduino_driver.pyから送られてくる速度指令をサブスクライブし、伊勢モードラにduty比を送る + 伊勢モードラからエンコーダの値を読み取り、rosにパブリッシュ + モーターの回転速度のPD制御を行う。  
 - MPU6050_manual_read  
