
#include <iomanip>

#include "ugv_sdk/mobile_robot/ranger_robot.hpp"

using namespace westonrobot;

int main(int argc, char *argv[]) {
  std::string device_name;

  if (argc == 2) {
    device_name = {argv[1]};
    std::cout << "Specified CAN: " << device_name << std::endl;
  } else {
    std::cout << "Usage: app_ranger_demo <interface>" << std::endl
              << "Example 1: ./app_ranger_demo can0" << std::endl;
    return -1;
  }

  RangerMiniV1Robot ranger;
  ranger.Connect(device_name);

  ranger.EnableCommandedMode();

  int count = 0;
  while (true) {
    if (count < 2000) {
      ranger.SetMotionMode(RangerInterface::MotionMode::kSpinning);
      ranger.SetMotionCommand(0.0, 0.0, -0.2);
    }

    auto state = ranger.GetRobotState();
    auto sensor = ranger.GetCommonSensorState();

    std::cout << "-------------------------------" << std::endl;
    std::cout << "count: " << count << std::endl;
    std::cout << "control mode: "
              << static_cast<int>(state.system_state.control_mode)
              << " , vehicle state: "
              << static_cast<int>(state.system_state.vehicle_state)
              << std::endl;
    std::cout << "battery voltage: " << state.system_state.battery_voltage
              << ", SOC: "
              << static_cast<int>(sensor.bms_basic_state.battery_soc)
              << std::endl;
    std::cout << "velocity (linear, angular, lateral, steering): "
              << std::setw(6) << state.motion_state.linear_velocity << ", "
              << std::setw(6) << state.motion_state.angular_velocity << ", "
              << std::setw(6) << state.motion_state.lateral_velocity << ", "
              << std::setw(6) << state.motion_state.steering_angle << std::endl;
    //        printf("velocity (linear, angular, lateral, steering): %.2f, %.2f,
    //        %.2f, "
    //               "%.2f\n",
    //               state.motion_state.linear_velocity,
    //               state.motion_state.angular_velocity,
    //               state.motion_state.lateral_velocity,
    //               state.motion_state.steering_angle);
    //        std::cout << "-------------------------------" << std::endl;

    usleep(20000);
    ++count;
  }
  return 0;
}
