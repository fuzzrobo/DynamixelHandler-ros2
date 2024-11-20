uint16[] id_list

# 書き込めない
string[] model
uint16[] firmware_version

# 書き込める
string[] protocol_version
dynamixel_handler/DynamixelExtraDrivemode drive_mode
dynamixel_handler/DynamixelExtraShutdown shutdown
dynamixel_handler/DynamixelExtraRestureconfig restore_configuration 
float64[] homing_offset_deg
float64[] return_delay_time_us
float64[] bus_watchbdog_ms          

#　機能
dynamixel_handler/DynamixelExtraLed led                    
uint8[] shadow_id
float64[] moving_threshold_deg_s
uint8[] status_return_level    

# 状況
dynamixel_handler/DynamixelExtraMovingstatus moving_status      
float64[] realtime_tick_us          
uint8[] moving                 
uint8[] registered_instruction 