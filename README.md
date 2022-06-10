# RoboCo

This is where I will be storing my notable scripts for RoboCo

# swerve_control.py

This is a controller for a drive train known as swerve. It currently supports:
 - Both Keyboard and Gamepad support for control
 - Any robot drive module configuration, so long as that configuration is given at startup
 - Speed modifiers to go slower or faster
 - Ghost position offsets, allowing the user to adjust the rotation point of the robot

![image](https://user-images.githubusercontent.com/24630044/172983614-af5a3f17-d965-4205-bcda-ea992b91faab.png)

Building a Swerve Module:
 - Each module consists of 2 motors, 1 imu, and 1 wheel
 - A Drive Motor is placed horizontal and a wheel is attached to it with a rod
 - A block strip is placed vertically above the Drive Motor whose end is flush with the top of the wheel
 - A 2x1 block strip is placed horizontally above both the wheel and block strip coming off the motor
 - A Rotate Motor is placed vertically above wheel on the block strip
 - An imu is placed next to the motor on the remaining spot on the strip
 - A rod is put into the Rotate Motor, allowing the module to attach to the bottom of your robot

![image](https://user-images.githubusercontent.com/24630044/172983470-98ef4dbb-062a-4c41-8c13-f411065a6289.png)

Assembling the Robot:
 - The Swerve Module can be copy pasted to create duplicates
 - Any amount of modules can be attached to the bottom base of a robot (4 in a rectangle config is generally good)
 - An imu needs to be placed on the upwards face of a block on your robot
   - This will act as the starting rotating point of the robot, but this location can be adjusted through controls
   - If you want to change the default rotating point, `RobotTargets.ghost_offset` holds the default (x,y)
 - A microcontroller needs to be placed on the robot (anywhere)
 - The script needs to point to `swerve_control.py`
 - The robot's imu and each module's imu and 2 motors need to be assigned ports

Configuring the Robot:
 - Scroll all the way down the script (`if __name__ == "__main__":`)
 - The `RobotConfig(...` needs to be told the port index of the main robot imu
 - Swerve Modules need to supply their x and y offset from the center of the robot, along with port indexes for the drive motor, rotate motor, and module imu
 - Controls can be edited below the Config if desired
