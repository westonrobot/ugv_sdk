import time
import scout_robot


def test_scout_robot():
    # Create an instance of ScoutRobot
    robot = scout_robot.ScoutRobot(scout_robot.ProtocolVersion.AGX_V1, True)

    # Connect to the robot
    try:
        robot.Connect("can0")
        print("Connection to the robot established.")
    except Exception as e:
        print(f"Failed to connect to the robot: {e}")
        return

    # Enable commanded mode
    robot.EnableCommandedMode()

    # Set motion command
    while True:
        robot.SetMotionCommand(1.0, 0.0)
        print("set motion command successfully.")
        time.sleep(20/1000)


if __name__ == "__main__":
    test_scout_robot()
