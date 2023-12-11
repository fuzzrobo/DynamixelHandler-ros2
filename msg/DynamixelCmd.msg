uint8[] ids           # モータID（Limbは1*2,1*3,1*4,車輪は2*1）  *はLimbの番号
float64[] positions     # 目標角度（Limb）
float64[] vel_profile   # 角速度（Limb）
float64[] velocities    # 目標速度（車輪）
float64[] current       
bool[]  torque_on     # トルクのON/OFF
bool[]  reboot        # リブート
float64[] errors