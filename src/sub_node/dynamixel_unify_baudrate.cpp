#include "dynamixel_communicator.h"
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
	// *** init node
	std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("dynamixel_unify_baudrate_node");


    int id_max, id_min;
    int baudrate_max, baudrate_min, baudrate_target; 
    int latency_timer;
	string device_name;
    nh->declare_parameter("min_id", 0);
    nh->declare_parameter("max_id", 50);
    nh->declare_parameter("device_name", "/dev/ttyUSB0");
    nh->declare_parameter("min_search_baudrate", 57600);
    nh->declare_parameter("max_search_baudrate", 4000000);
    nh->declare_parameter("target_baudrate", 1000000);
    nh->declare_parameter("latency_timer", 16);

    id_min = nh->get_parameter("min_id").as_int();
    id_max = nh->get_parameter("max_id").as_int();
    device_name = nh->get_parameter("device_name").as_string();
    baudrate_min = nh->get_parameter("min_search_baudrate").as_int();
    baudrate_max = nh->get_parameter("max_search_baudrate").as_int();
    baudrate_target = nh->get_parameter("target_baudrate").as_int();
    latency_timer = nh->get_parameter("latency_timer").as_int();

    // if (!nh->get_parameter("min_id", id_min )) id_min =  0;
    // if (!nh->get_parameter("max_id", id_max )) id_max =  50;
    // if (!nh->get_parameter("device_name", device_name    )) device_name = "/dev/ttyUSB0";
    // if (!nh->get_parameter("min_search_baudrate", baudrate_min)) baudrate_min = 57600;
    // if (!nh->get_parameter("max_search_baudrate", baudrate_max)) baudrate_max = 4000000;
    // if (!nh->get_parameter("target_baudrate",baudrate_target)) baudrate_target = 1000000;
    // if (!nh->get_parameter("latency_timer", latency_timer)) latency_timer = 16;

    auto dyn_comm = DynamixelCommunicator(); fflush(stdout);
    dyn_comm.GetPortHandler(device_name.c_str()); fflush(stdout);
    dyn_comm.set_retry_config(5, 20);  fflush(stdout); // printfのバッファを吐き出す． これがないと printfの表示が遅延する

    uint64_t dyn_baudrate;
    switch (baudrate_target) {
        case 9600:    dyn_baudrate = BAUDRATE_INDEX_9600;   break;
        case 57600:   dyn_baudrate = BAUDRATE_INDEX_57600;  break;
        case 115200:  dyn_baudrate = BAUDRATE_INDEX_115200; break;
        case 1000000: dyn_baudrate = BAUDRATE_INDEX_1M;     break;
        case 2000000: dyn_baudrate = BAUDRATE_INDEX_2M;     break;
        case 3000000: dyn_baudrate = BAUDRATE_INDEX_3M;     break;
        case 4000000: dyn_baudrate = BAUDRATE_INDEX_4M;     break;
        case 4500000: dyn_baudrate = BAUDRATE_INDEX_4M5;    break;
        case 6000000: dyn_baudrate = BAUDRATE_INDEX_6M;     break;
        case 10500000:dyn_baudrate = BAUDRATE_INDEX_10M5;   break;
        default: RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Invalid baudrate %d", baudrate_target); return false;
    }

    vector<int> baudrate_list = {
        9600,
        57600,
        115200,
        1000000,
        2000000,
        3000000,
        4000000,
        4500000,
        6000000,
        10500000
    };

    vector<int> found_ids;
    for ( auto br : baudrate_list ) {
        if ( br < baudrate_min || br > baudrate_max) continue;
        dyn_comm.set_baudrate(br); fflush(stdout);
        dyn_comm.set_latency_timer( (br<=57600) ? 16 : latency_timer); fflush(stdout);
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Searching Dynamixel baudrate [%d] ...", br);
        if ( !dyn_comm.OpenPort() ) { RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open"); continue; }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " ...");
        for (int i=id_min; i<=id_max; i++) {
            if ( std::find(found_ids.begin(), found_ids.end(), i) != found_ids.end() ) continue;
            if ( !dyn_comm.tryPing(i) ) continue;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ID [%d] is found, try change baudrate %d", i, baudrate_target);
            auto dyn_model = dyn_comm.tryRead(AddrCommon::model_number, i);
            switch ( dynamixel_series(dyn_model) ) {
                case SERIES_X: 
                    dyn_comm.tryWrite(AddrX::torque_enable, i, TORQUE_DISABLE);
                    dyn_comm.tryWrite(AddrX::baudrate, i, dyn_baudrate);
                    break;
                case SERIES_P:
                    dyn_comm.tryWrite(AddrP::torque_enable, i, TORQUE_DISABLE);
                    dyn_comm.tryWrite(AddrP::baudrate, i, dyn_baudrate);
                    break;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Unknown series %ld", dyn_model);
                    continue;
            } 
            found_ids.push_back(i);
        }
        dyn_comm.ClosePort(); fflush(stdout);
    }

    dyn_comm.set_baudrate(baudrate_target); fflush(stdout);
    dyn_comm.set_latency_timer( (baudrate_target<=57600) ? 16 : latency_timer); fflush(stdout);
    if ( !dyn_comm.OpenPort() ) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to open USB device [%s]", dyn_comm.port_name().c_str()); 
        return false;
    }

    for ( auto i : found_ids ) {
        if ( dyn_comm.tryPing(i) )
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ID [%d] is succeded to change baudrate %d", i, baudrate_target);
        else
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ID [%d] is failed to change baudrate", i);
    }

    rclcpp::shutdown();
}