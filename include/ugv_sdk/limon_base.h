#ifndef LIMON_BASE_H
#define LIMON_BASE_H

#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <queue>

#include "ugv_sdk/agilex_base_serialport.h"

#define MAX_SERIAL_BUFFER_SIZE 2048

namespace westonrobot {
struct LimonState {
  // system state
  SystemStateMessage system_state;
  MotionStateMessage motion_state;
  LightStateMessage light_state;

  RcStateMessage rc_state;

  ActuatorHSStateMessage actuator_hs_state[8];
  ActuatorLSStateMessage actuator_ls_state[8];
  MotionModeFeedbackMessage current_motion_mode;

  // sensor data
  OdometryMessage odometry;
};

struct LimonMotionCmd {
  double linear_velocity;
  double angular_velocity;
};

struct LimonLightCmd {
  LimonLightCmd() = default;
  LimonLightCmd(LightMode f_mode, uint8_t f_value, LightMode r_mode,
                uint8_t r_value)
      : cmd_ctrl_allowed(true),
        front_mode(f_mode),
        front_custom_value(f_value),
        rear_mode(r_mode),
        rear_custom_value(r_value) {}

  bool cmd_ctrl_allowed = false;
  LightMode front_mode;
  uint8_t front_custom_value;
  LightMode rear_mode;
  uint8_t rear_custom_value;
};

/////////////////////////////////////////////////////////////////////////

class LimonBase : public AgilexBaseSerialPort {
 public:
  LimonBase() : AgilexBaseSerialPort(){};
  ~LimonBase() = default;

  // set up connection
  void Connect(std::string dev_name, uint32_t bouadrate) override;

  // robot control
  void SetMotionCommand(double linear_vel, double steer_angle,
                        double lateral_vel = 0.0, double angular_vel = 0.0);
  void SetLightCommand(const LimonLightCmd &cmd);
  void SetMotionMode(uint8_t mode);

  // get robot state
  LimonState GetLimonState();
  void ParseSerialFrame(uint8_t *data, const size_t bufsize, size_t len);


 private:
  LimonState limon_state_;

  void ParseCANFrame(can_frame *rx_frame);
  void UpdateLimonState(const AgxMessage &status_msg, LimonState &state);

    std::mutex serial_callback_mutex_;
  std::queue<uint8_t> serial_raw_data_;
};
}  // namespace westonrobot
#endif  // LIMON_BASE_H
