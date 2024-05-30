import time
import ugv_sdk_py
from ugv_sdk_py import hunter_robot


def test_hunter_robot():
    # Create an instance of HunterRobot
    robot = hunter_robot.HunterRobot(ugv_sdk_py.ProtocolVersion.AGX_V2)

    # Connect to the robot
    try:
        robot.connect("can0")
        print("Connection to the robot established.")
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")
        return
    
    print("protocol version: ", robot.get_parser_protocol_version())
    print("robot version: ", robot.request_version())

    # Enable commanded mode
    robot.enable_commanded_mode()

    # Set motion command
    while True:
        # robot control
        robot.set_motion_command(0.0, 0.0)

        # hunter core state
        state = robot.get_robot_state()
        print("time: ", state.time_stamp)
        print("system_state: ")
        print(" - vehicle_state: ", state.system_state.vehicle_state)
        print(" - control_mode: ", state.system_state.control_mode)
        print(" - battery voltage: ", state.system_state.battery_voltage)
        print(" - error_code: ", state.system_state.error_code)
        print(
            "motion state: {0}, {1}, {2}, {3}".format(
                state.motion_state.linear_velocity,
                state.motion_state.angular_velocity,
                state.motion_state.lateral_velocity,
                state.motion_state.steering_angle
            )
        )
        print(
            "rc state: {0}, {1}, {2}, {3}; {4}, {5}, {6}, {7}; {8}".format(
                state.rc_state.swa,
                state.rc_state.swb,
                state.rc_state.swc,
                state.rc_state.swd,
                state.rc_state.stick_right_v,
                state.rc_state.stick_right_h,
                state.rc_state.stick_left_v,
                state.rc_state.stick_left_h,
                state.rc_state.var_a
            )
        )

        # actuator state
        actuator_state = robot.get_actuator_state()
        for i in range(3):
            print(
                "actuator {0} hs: {1}, {2}, {3}, {4}".format(
                    i,
                    actuator_state.actuator_hs_state[i].motor_id,
                    actuator_state.actuator_hs_state[i].rpm,
                    actuator_state.actuator_hs_state[i].current,
                    actuator_state.actuator_hs_state[i].pulse_count
                )
            )
            print(
                "actuator {0} ls: {1}, {2}, {3}, {4}".format(
                    i,
                    actuator_state.actuator_ls_state[i].driver_voltage,
                    actuator_state.actuator_ls_state[i].driver_temp,
                    actuator_state.actuator_ls_state[i].motor_temp,
                    actuator_state.actuator_ls_state[i].driver_state
                )
            )

        # common sensor state
        common_sensor = robot.get_common_sensor_state()
        print(
            "bms_basic: voltage {0}, current {1}, temperature {2}, soc {3}, soh {4}".format(
                common_sensor.bms_basic_state.voltage,
                common_sensor.bms_basic_state.current,
                common_sensor.bms_basic_state.temperature,
                common_sensor.bms_basic_state.battery_soc,
                common_sensor.bms_basic_state.battery_soh
            )
        )
        # print("bms_extend_state: ")
        # print(" - alarm_status_1: ", common_sensor.bms_extend_state.alarm_status_1)
        # print(" - alarm_status_2: ", common_sensor.bms_extend_state.alarm_status_2)
        # print(" - warn_status_1: ", common_sensor.bms_extend_state.warn_status_1)
        # print(" - warn_status_2: ", common_sensor.bms_extend_state.warn_status_2)

        time.sleep(0.1)


if __name__ == "__main__":
    test_hunter_robot()