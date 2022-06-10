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
 - A microcontroller needs to be placed on the robot (anywhere)
 - The script needs to point to `swerve_control.py`
 - The robot's imu and each module's imu and 2 motors need to be assigned ports

Configuring the Robot:
 - Scroll all the way down the script (`if __name__ == "__main__":`)
 - The `SwerveController(...` will hold your robot configuration
 - `imu_pin` needs to be supplied the pin for the imu of the robot itself
 - `modules` needs to be supplied a list of Swerve Modules attached to the robot
   - `x` and `y` hold the offset (in blocks) from the center of the robot to the module
   - `drive_pin`, `rot_pin`, and `imu_pin` hold the pins for the 2 motors and imu on the module
 - Controls can be edited below if desired
