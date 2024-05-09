import time
import scout_robot


def test_scout_robot():
    # Create an instance of ScoutRobot
    robot = scout_robot.ScoutRobot(scout_robot.ProtocolVersion.AGX_V2, True)

    # Connect to the robot
    try:
        robot.connect("can0")
        print("Connection to the robot established.")
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")
        return

    # Enable commanded mode
    robot.enable_commanded_mode()

    # Set motion command
    while True:
        # robot control
        robot.set_motion_command(1.0, 0.0)

        # robot state monitoring
        state = robot.get_robot_state()
        print("time: ", state.time_stamp)
        print("battery voltage: ", state.system_state.battery_voltage)
        print(
            "motion state: {0}, {1}, {2}, {3}".format(
                state.motion_state.linear_velocity,
                state.motion_state.angular_velocity,
                state.motion_state.lateral_velocity,
                state.motion_state.steering_angle,
            )
        )
        print("rc state: {0}, {1}, {2}, {3}; {4}, {5}, {6}, {7}; {8}".format(
            state.rc_state.swa,
            state.rc_state.swb,
            state.rc_state.swc,
            state.rc_state.swd,
            state.rc_state.stick_right_v,
            state.rc_state.stick_right_h,
            state.rc_state.stick_left_v,
            state.rc_state.stick_left_h,
            state.rc_state.var_a
        ))

        time.sleep(20 / 1000)


if __name__ == "__main__":
    test_scout_robot()
