from inputs import Input
from controllables import DCMotor
from sensors import InertialMotionUnit
from routines import Vector3, Matrix4x4
from time import time_ns
import math

# Boiler plate to interface with a gamepad through windows
from ctypes import WinDLL, WinError, Structure, POINTER, byref, c_ubyte
from ctypes.util import find_library
from ctypes.wintypes import DWORD, WORD, SHORT
BYTE = c_ubyte
XUSER_MAX_COUNT = 4
MAX_JOY_VAL = math.pow(2, 15)
MAX_TRIG_VAL = math.pow(2, 8)
class XINPUT_BUTTONS(Structure):
    _fields_ = [("DPAD_UP", WORD, 1), ("DPAD_DOWN", WORD, 1), ("DPAD_LEFT", WORD, 1), ("DPAD_RIGHT", WORD, 1), ("START", WORD, 1), ("BACK", WORD, 1), ("LEFT_THUMB", WORD, 1), ("RIGHT_THUMB", WORD, 1), ("LEFT_SHOULDER", WORD, 1), ("RIGHT_SHOULDER", WORD, 1), ("_reserved_1_", WORD, 1), ("_reserved_1_", WORD, 1), ("A", WORD, 1), ("B", WORD, 1), ("X", WORD, 1), ("Y", WORD, 1)]
    def __repr__(self):
        r = []
        for name, type, size in self._fields_:
            if "reserved" in name:
                continue
            r.append("{}={}".format(name, getattr(self, name)))
        args = ', '.join(r)
        return f"XINPUT_GAMEPAD({args})"
class XINPUT_GAMEPAD(Structure):
    _fields_ = [("wButtons", XINPUT_BUTTONS), ("bLeftTrigger", BYTE), ("bRightTrigger", BYTE), ("sThumbLX", SHORT), ("sThumbLY", SHORT), ("sThumbRX", SHORT), ("sThumbRY", SHORT),]
    def __repr__(self):
        r = []
        for name, type in self._fields_:
            r.append("{}={}".format(name, getattr(self, name)))
        args = ', '.join(r)
        return f"XINPUT_GAMEPAD({args})"
class XINPUT_STATE(Structure):
    _fields_ = [("dwPacketNumber", DWORD), ("Gamepad", XINPUT_GAMEPAD)]
    def __repr__(self):
        return f"XINPUT_STATE(dwPacketNumber={self.dwPacketNumber}, Gamepad={self.Gamepad})"
class XInput:
    def __init__(self):
        lib_name = "XInput1_4.dll"  
        lib_path = find_library(lib_name)
        if not lib_path:
            raise Exception(f"Couldn't find {lib_name}")
        self._XInput_ = WinDLL(lib_path)
        self._XInput_.XInputGetState.argtypes = [DWORD, POINTER(XINPUT_STATE)]
        self._XInput_.XInputGetState.restype = DWORD
    def GetState(self, dwUserIndex):
        state = XINPUT_STATE()
        ret = self._XInput_.XInputGetState(dwUserIndex, byref(state))
        if ret:
            raise WinError(ret)
        return state.dwPacketNumber, state.Gamepad

# Wrapper to interface with the gamepad cleanly
class Gamepad:
    trig_thresh = 0.5
    keys = ['dpad_up', 'dpad_down', 'dpad_left', 'dpad_right', 'start', 'back', 'left_stick', 'right_stick', 'left_bumper', 'right_bumper', 'a', 'b', 'x', 'y', 'left_trigger', 'right_trigger', 'left_joy_x', 'left_joy_y', 'right_joy_x', 'right_joy_y']

    def __init__(self, gamepad_index=0):
        self.gamepad_index = gamepad_index
        self.xi = XInput()
        try:
            self.xi.GetState(self.gamepad_index)
            self.has_controller = True
            print('Controller found connnected!')
        except:
            self.has_controller = False
            print('No controller found.....')
    
    def get_button(self, key):
        if not self.has_controller:
            return False
        action_map = {
            'dpad_up':       lambda x: x.wButtons.DPAD_UP > 0,
            'dpad_down':     lambda x: x.wButtons.DPAD_DOWN > 0,
            'dpad_left':     lambda x: x.wButtons.DPAD_LEFT > 0,
            'dpad_right':    lambda x: x.wButtons.DPAD_RIGHT > 0,
            'start':         lambda x: x.wButtons.START > 0,
            'back':          lambda x: x.wButtons.BACK > 0,
            'left_stick':    lambda x: x.wButtons.LEFT_THUMB > 0,
            'right_stick':   lambda x: x.wButtons.RIGHT_THUMB > 0,
            'left_bumper':   lambda x: x.wButtons.LEFT_SHOULDER > 0,
            'right_bumper':  lambda x: x.wButtons.RIGHT_SHOULDER > 0,
            'a':             lambda x: x.wButtons.A > 0,
            'b':             lambda x: x.wButtons.B > 0,
            'x':             lambda x: x.wButtons.X > 0,
            'y':             lambda x: x.wButtons.Y > 0,
            'left_trigger':  lambda x: x.bLeftTrigger / MAX_TRIG_VAL > self.trig_thresh,
            'right_trigger': lambda x: x.bRightTrigger / MAX_TRIG_VAL > self.trig_thresh,
        }
        return action_map[key](self.xi.GetState(self.gamepad_index)[1])

    def get_axis(self, key):
        if not self.has_controller:
            return 0.
        action_map = {
            'left_trigger':  lambda x: x.bLeftTrigger / MAX_TRIG_VAL,
            'right_trigger': lambda x: x.bRightTrigger / MAX_TRIG_VAL,
            'left_joy_x':    lambda x: x.sThumbLX / MAX_JOY_VAL,
            'left_joy_y':    lambda x: x.sThumbLY / MAX_JOY_VAL,
            'right_joy_x':   lambda x: x.sThumbRX / MAX_JOY_VAL,
            'right_joy_y':   lambda x: x.sThumbRY / MAX_JOY_VAL,
        }
        return action_map[key](self.xi.GetState(self.gamepad_index)[1])

def get_time():
    return time_ns() / (10 ** 9)

# Class for calculating the PID control (used for module rotation)
class PID:
    previous_error, integral = 0, 0

    def __init__(self, Kp, Ki, Kd, dead_zone=0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dead_zone = dead_zone
        self.prev_time = get_time()
    
    def get_power(self, error):
        curr_time = get_time()
        dt = curr_time - self.prev_time

        self.integral = self.integral + error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.previous_error = error
        self.prev_time = curr_time

        if abs(error) < self.dead_zone:
            return 0
        return output

class Helpers:
    def get_heading(pin, axis=lambda v: v.y):
        return axis(Vector3(Matrix4x4.from_rotation(InertialMotionUnit(pin).rotation_axis())._backing.rotation.eulerAngles))

    def angle_between(angle1, angle2):
        diff = angle1 - angle2
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff

# Class for holding information on how the drive motor(s) are configured
class ModuleDriver:
    def __init__(self, pin, reversed=False, counter_rotate=False):
        self.pin = pin
        self.reversed = reversed
        self.counter_rotate = counter_rotate

# Class for holding information about module placement and connection on the robot
class SwerveModule:
    def __init__(self, x, y, drivers, rot_pin, imu_pin, imu_read=lambda v: v.y):
        self.x = x
        self.y = y
        self.drivers = drivers
        if isinstance(self.drivers, ModuleDriver):
            self.drivers = [self.drivers]
        self.rot_pin = rot_pin
        self.imu_pin = imu_pin
        self.imu_read = imu_read

# Main class that handles all logic from processing user input to applying motor speed
class SwerveController:
    def __init__(self, imu_pin, modules, keyboard_controls, gamepad_controls, pid_setup={'Kp': 1, 'Ki': 0, 'Kd': 0, 'dead_zone': 0.02}):
        self.imu_pin = imu_pin
        self.modules = modules
        self.module_count = len(modules)
        self.keyboard_controls = keyboard_controls
        self.gamepad_controls  = gamepad_controls
        self.gamepad = Gamepad()
        self.target_vels = [0 for _ in range(self.module_count)]
        self.target_rots = [0 for _ in range(self.module_count)]
        self.pids = [PID(Kp=pid_setup['Kp'], Ki=pid_setup['Ki'], Kd=pid_setup['Kd'], dead_zone=pid_setup['dead_zone']) for _ in range(self.module_count)]

    def step(self):
        self.mode_controls()
        self.ghost_controls()
        self.speed_controls()

        drive, strafe, turn = self.get_movement()
        if self.curr_mode == 0:
            self.direct_drive(drive, strafe, turn)
        elif self.curr_mode == 1:
            self.gps_drive(drive, strafe, turn)
        else:
            print(f'Robot currently in unsupported mode! Reverting back to mode 0 ({self.mode_names[0]})!')
            self.curr_mode = 0
        self.apply_targets()

    prev_mode_mod = False
    curr_mode, mode_count = 0, 1
    mode_names = ['Direct Drive']
    def mode_controls(self):
        swap_modes = Input.pressed(self.keyboard_controls['swap_modes']) or self.gamepad.get_button(self.gamepad_controls['swap_modes'])
        if not self.prev_mode_mod and swap_modes:
            self.curr_mode += 1
            if self.curr_mode >= self.mode_count:
                self.curr_mode = 0
            print(f'Swapped to Mode: {self.mode_names[self.curr_mode]}')
        self.prev_mode_mod = swap_modes

    prev_ghost_mod = False
    ghost_offset = [0, 0]
    def ghost_controls(self):
        ghost_up    = Input.pressed(self.keyboard_controls['ghost_up'])    or self.gamepad.get_button(self.gamepad_controls['ghost_up'])
        ghost_down  = Input.pressed(self.keyboard_controls['ghost_down'])  or self.gamepad.get_button(self.gamepad_controls['ghost_down'])
        ghost_left  = Input.pressed(self.keyboard_controls['ghost_left'])  or self.gamepad.get_button(self.gamepad_controls['ghost_left'])
        ghost_right = Input.pressed(self.keyboard_controls['ghost_right']) or self.gamepad.get_button(self.gamepad_controls['ghost_right'])
        if not self.prev_ghost_mod:
            flag = False
            if ghost_up:
                self.ghost_offset[1] += 1
            elif ghost_down:
                self.ghost_offset[1] -= 1
            elif ghost_left:
                self.ghost_offset[0] -= 1
            elif ghost_right:
                self.ghost_offset[0] += 1
            else:
                flag = True
            if not flag:
                print(f'New Ghost Offset: [x = {self.ghost_offset[0]}, y = {self.ghost_offset[1]}]')
        self.prev_ghost_mod = ghost_up or ghost_down or ghost_left or ghost_right

    prev_speed_mod = False
    curr_pow = 0.5
    MIN_POW, MAX_POW = 0.1, 1.0
    def speed_controls(self):
        speed_up   = Input.pressed(self.keyboard_controls['speed_up'])   or self.gamepad.get_button(self.gamepad_controls['speed_up'])
        speed_down = Input.pressed(self.keyboard_controls['speed_down']) or self.gamepad.get_button(self.gamepad_controls['speed_down'])
        if not self.prev_speed_mod:
            if speed_up:
                self.curr_pow += 0.1
                self.curr_pow = min(self.MAX_POW, round(self.curr_pow, 1))
                print(f'Set robot power to: {self.curr_pow}')
            elif speed_down:
                self.curr_pow -= 0.1
                self.curr_pow = max(self.MIN_POW, round(self.curr_pow, 1))
                print(f'Set robot power to: {self.curr_pow}')
        self.prev_speed_mod = speed_up or speed_down

    controller_dead_zone = 0.1
    def get_movement(self):
        drive  = self.gamepad.get_axis(self.gamepad_controls['drive'])
        strafe = self.gamepad.get_axis(self.gamepad_controls['strafe'])
        turn   = self.gamepad.get_axis(self.gamepad_controls['turn'])
        if abs(drive)  < self.controller_dead_zone: drive  = 0
        if abs(strafe) < self.controller_dead_zone: strafe = 0
        if abs(turn)   < self.controller_dead_zone: turn   = 0

        if Input.pressed(self.keyboard_controls['drive_forward']):
            drive += 1
        if Input.pressed(self.keyboard_controls['drive_back']):
            drive -= 1
        if Input.pressed(self.keyboard_controls['strafe_left']):
            strafe -= 1
        if Input.pressed(self.keyboard_controls['strafe_right']):
            strafe += 1
        if Input.pressed(self.keyboard_controls['turn_left']):
            turn -= 1
        if Input.pressed(self.keyboard_controls['turn_right']):
            turn += 1
        
        return drive, strafe, turn

    def direct_drive(self, drive, strafe, turn):
        module_range = range(self.module_count)
        y_comp = [self.modules[i].y - self.ghost_offset[1] for i in module_range]
        x_comp = [self.modules[i].x - self.ghost_offset[0] for i in module_range]
        angles = [math.atan2(y_comp[i], x_comp[i])+math.pi/2 for i in module_range]

        dists = [math.pow(y_comp[i]**2 + x_comp[i]**2, 0.5) for i in module_range]
        max_dist = max(dists)
        max_dist = 1 if max_dist < 1 else max_dist
        mods = [dists[i] / max_dist for i in module_range]

        turns_y = [mods[i] * -turn * math.sin(angles[i]) for i in module_range]
        turns_x = [mods[i] * -turn * math.cos(angles[i]) for i in module_range]

        y = [drive + turns_y[i] for i in module_range]
        x = [strafe + turns_x[i] for i in module_range]

        self.target_vels = [pow(y[i]**2 + x[i]**2, 0.5) for i in module_range]
        new_rots = [(math.degrees(math.atan2(y[i], x[i]))-90 if (x != 0 and y != 0) else self.target_rots[i]) for i in module_range]
        for i in module_range:
            if abs(self.target_vels[i]) < 0.01:
                continue
            diff = self.target_rots[i] - new_rots[i]
            if diff > 90:
                self.target_rots[i] = new_rots[i] + 180
                self.target_vels[i] *= -1
            elif diff < -90:
                self.target_rots[i] = new_rots[i] - 180
                self.target_vels[i] *= -1
            else:
                self.target_rots[i] = new_rots[i]

    TURN_PROP = 1.0
    def apply_targets(self):
        module_range = range(self.module_count)
        raw_rots = [Helpers.get_heading(self.modules[i].imu_pin, axis=self.modules[i].imu_read) for i in module_range]
        robot_heading = Helpers.get_heading(self.imu_pin)
        rots = [robot_heading - raw_rots[i] for i in module_range]
        diffs = [max(-1, min(1, -Helpers.angle_between(rots[i], self.target_rots[i]) / 180. * 2)) for i in module_range]
        delta_rot = [self.pids[i].get_power(diffs[i]) for i in module_range]
        
        for i in module_range:
            drivers = self.modules[i].drivers
            rot_motor = DCMotor(self.modules[i].rot_pin)

            target_vel = self.target_vels[i] * self.curr_pow
            for driver in drivers:
                target_vel_2 = target_vel
                if driver.counter_rotate:
                    target_vel_2 += delta_rot[i] * (-1 if driver.reversed else 1) * self.TURN_PROP
                DCMotor(driver.pin).flipped = (target_vel_2 < 0) != (driver.reversed)
                DCMotor(driver.pin).spin(abs(target_vel_2))

            rot_motor.flipped = delta_rot[i] < 0
            rot_motor.spin(abs(delta_rot[i]))

if __name__ == "__main__":
    swerve_controller = SwerveController(

        imu_pin=12,
        modules=[
            SwerveModule(x=-12, y=-4, drivers=ModuleDriver(pin=0),  rot_pin=4,  imu_pin=8),  # BL
            SwerveModule(x=12,  y=-4, drivers=ModuleDriver(pin=1),  rot_pin=5,  imu_pin=9),  # BR
            SwerveModule(x=-12, y=4,  drivers=ModuleDriver(pin=2),  rot_pin=6,  imu_pin=10), # FL
            SwerveModule(x=12,  y=4,  drivers=ModuleDriver(pin=3),  rot_pin=7,  imu_pin=11), # FR
            SwerveModule(x=0,   y=-7, drivers=ModuleDriver(pin=13), rot_pin=14, imu_pin=15), # BC
            SwerveModule(x=0,   y=7,  drivers=ModuleDriver(pin=16), rot_pin=17, imu_pin=18), # FC
        ],
        pid_setup={'Kp': 1, 'Ki': 0, 'Kd': 0, 'dead_zone': 0.01},

        # imu_pin=24,
        # modules=[
        #     SwerveModule(x=-12, y=4,  drivers=[ModuleDriver(0,  True, True), ModuleDriver(1,  False, True)], rot_pin=2,  imu_pin=3,  imu_read=lambda v: -v.y), # FL
        #     SwerveModule(x=0,   y=7,  drivers=[ModuleDriver(4,  True, True), ModuleDriver(5,  False, True)], rot_pin=6,  imu_pin=7,  imu_read=lambda v: -v.y), # FC
        #     SwerveModule(x=12,  y=4,  drivers=[ModuleDriver(8,  True, True), ModuleDriver(9,  False, True)], rot_pin=10, imu_pin=11, imu_read=lambda v: -v.y), # FR
        #     SwerveModule(x=-12, y=-4, drivers=[ModuleDriver(12, True, True), ModuleDriver(13, False, True)], rot_pin=14, imu_pin=15, imu_read=lambda v: -v.y), # BL
        #     SwerveModule(x=0,   y=-7, drivers=[ModuleDriver(16, True, True), ModuleDriver(17, False, True)], rot_pin=18, imu_pin=19, imu_read=lambda v: -v.y), # BC
        #     SwerveModule(x=12,  y=-4, drivers=[ModuleDriver(20, True, True), ModuleDriver(21, False, True)], rot_pin=22, imu_pin=23, imu_read=lambda v: -v.y), # BR
        # ],
        # pid_setup={'Kp': 1, 'Ki': 0, 'Kd': 0, 'dead_zone': 0.01},

        keyboard_controls={
            "drive_forward": "w",
            "drive_back":    "s",
            "strafe_left":   "a",
            "strafe_right":  "d",
            "turn_left":     "q",
            "turn_right":    "e",
            "swap_modes":    "t",
            "ghost_up":      "i",
            "ghost_down":    "k",
            "ghost_left":    "j",
            "ghost_right":   "l",
            "speed_up":      "x",
            "speed_down":    "z",
        },
        gamepad_controls={
            "drive":       "left_joy_y",
            "strafe":      "left_joy_x",
            "turn":        "right_joy_x",
            "swap_modes":  "back",
            "ghost_up":    "dpad_up",
            "ghost_down":  "dpad_down",
            "ghost_left":  "dpad_left",
            "ghost_right": "dpad_right",
            "speed_up":    "left_bumper",
            "speed_down":  "left_trigger",
        },
    )
    while True:
        swerve_controller.step()