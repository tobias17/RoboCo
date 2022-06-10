from inputs import *
from controllables import *
from sensors import *
from scriptruntime import *
from ports import *
from color import *
from routines import *
from math import tau
from time import sleep

from ctypes import WinDLL, WinError, Structure, POINTER, byref, c_ubyte
from ctypes.util import find_library
from ctypes.wintypes import DWORD, WORD, SHORT

# TODO List:
# 2. Different navigation methods

# Class for holding information about module placement and connection on the robot
class SwerveModule():
    def __init__(self, x, y, drive_pin, rot_pin, imu_pin):
        self.x = x
        self.y = y
        self.drive_pin = drive_pin
        self.rot_pin = rot_pin
        self.imu_pin = imu_pin

# Class for holding information about the entire robot
class RobotConfig():
    def __init__(self, imu_pin, modules):
        self.imu_pin = imu_pin
        self.modules = modules
        self.count = len(modules)

    def read_imu_angles(self):
        rots = [get_heading(self.modules[i].imu_pin) for i in range(self.count)]
        robot_heading = get_heading(self.imu_pin)
        rots_norm = [robot_heading - rots[i] for i in range(self.count)]
        return rots_norm

    def apply_targets(self):
        rots = self.read_imu_angles()
        diffs = [-angle_between(rots[i], RobotTargets.rots[i]) / 180. for i in range(self.count)]

        drive_motors = [controllables.DCMotor(self.modules[i].drive_pin) for i in range(self.count)]
        rot_motors   = [controllables.DCMotor(self.modules[i].rot_pin) for i in range(self.count)]
        for i in range(self.count):
            target_vel = RobotTargets.vels[i] * RobotTargets.pow
            drive_motors[i].flipped = target_vel < 0
            drive_motors[i].spin(target_vel * sign_of(target_vel))
            rot_motors[i].flipped = diffs[i] < 0
            rot_motors[i].spin(diffs[i] * sign_of(diffs[i]))

MAX_JOY_VAL = math.pow(2, 15)
MAX_TRIG_VAL = math.pow(2, 8)
MIN_POW = 0.1
MAX_POW = 1

# Reads in the controller / keyboard input, applies robot movement targets to the RobotTargets static class
class ControlScheme():
    prev_state_mod = False
    prev_ghost_mod = False
    prev_speed_mod = False
    show_speeds = False
    controller_dead_zone = 0.1

    def __init__(self, gamepad_controls, keyboard_controls):
        self.gamepad_controls = gamepad_controls
        self.keyboard_controls = keyboard_controls
        self.xi = XInput()
        try:
            self.xi.GetState(0)
            self.has_controller = True
            print('Controller found connnected!')
        except:
            self.has_controller = False
            print('No controller found.....')

    def read_controls(self, config):
        self.state_controls()
        self.ghost_controls()
        self.speed_controls()
        self.drive_controls(config)
    
    def state_controls(self):
        if self.has_controller:
            ctrl = self.xi.GetState(0)[1]
        swap_modes = Input.pressed(self.keyboard_controls['swap_modes']) or (self.has_controller and self.gamepad_controls['swap_modes'](ctrl))
        if not self.prev_state_mod and swap_modes:
            RobotTargets.curr_state += 1
            if RobotTargets.curr_state >= RobotTargets.state_count:
                RobotTargets.curr_state = 0
            print(f'Swapped to Mode: {RobotTargets.state_name()}')
        self.prev_state_mod = swap_modes

    def ghost_controls(self):
        if self.has_controller:
            ctrl = self.xi.GetState(0)[1]
        ghost_up    = Input.pressed(self.keyboard_controls['ghost_up'])    or (self.has_controller and self.gamepad_controls['ghost_up'](ctrl))
        ghost_down  = Input.pressed(self.keyboard_controls['ghost_down'])  or (self.has_controller and self.gamepad_controls['ghost_down'](ctrl))
        ghost_left  = Input.pressed(self.keyboard_controls['ghost_left'])  or (self.has_controller and self.gamepad_controls['ghost_left'](ctrl))
        ghost_right = Input.pressed(self.keyboard_controls['ghost_right']) or (self.has_controller and self.gamepad_controls['ghost_right'](ctrl))
        if not self.prev_ghost_mod:
            flag = False
            if ghost_up:
                RobotTargets.ghost_offset[1] += 1
            elif ghost_down:
                RobotTargets.ghost_offset[1] -= 1
            elif ghost_left:
                RobotTargets.ghost_offset[0] -= 1
            elif ghost_right:
                RobotTargets.ghost_offset[0] += 1
            else:
                flag = True
            if not flag:
                print(f'New Ghost Offset: [x = {RobotTargets.ghost_offset[0]}, y = {RobotTargets.ghost_offset[1]}]')
        self.prev_ghost_mod = ghost_up or ghost_down or ghost_left or ghost_right

    def speed_controls(self):
        if self.has_controller:
            ctrl = self.xi.GetState(0)[1]
        speed_up   = Input.pressed(self.keyboard_controls['speed_up'])   or (self.has_controller and self.gamepad_controls['speed_up'](ctrl))
        speed_down = Input.pressed(self.keyboard_controls['speed_down']) or (self.has_controller and self.gamepad_controls['speed_down'](ctrl))
        if not self.prev_speed_mod:
            if speed_up:
                RobotTargets.pow += 0.1
                RobotTargets.pow = min(MAX_POW, round(RobotTargets.pow, 1))
                print(f'Set robot power to: {RobotTargets.pow}')
            elif speed_down:
                RobotTargets.pow -= 0.1
                RobotTargets.pow = max(MIN_POW, round(RobotTargets.pow, 1))
                print(f'Set robot power to: {RobotTargets.pow}')
        self.prev_speed_mod = speed_up or speed_down

        # Keyboard
        for i in range(10):
            if Input.pressed(f'{i}'):
                RobotTargets.pow = (1 if i == 0 else i/10)
                print(f'Set robot power to: {RobotTargets.pow}')
    
    def drive_controls(self, config):
        drive, strafe, turn = 0, 0, 0

        # Controller
        if self.has_controller:
            ctrl = self.xi.GetState(0)[1]
            drive, strafe, turn = self.gamepad_controls['drive'](ctrl), self.gamepad_controls['strafe'](ctrl), self.gamepad_controls['turn'](ctrl)
            if abs(drive)  < self.controller_dead_zone: drive = 0
            if abs(strafe) < self.controller_dead_zone: strafe = 0
            if abs(turn)   < self.controller_dead_zone: turn = 0

        # Keyboard
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

        if self.show_speeds: print(f'Drive: {drive:.2f}, Strafe: {strafe:.2f}, Turn: {turn:.2f}')
        
        if RobotTargets.curr_state == 0:
            y_comp = [config.modules[i].y - RobotTargets.ghost_offset[1] for i in range(config.count)]
            x_comp = [config.modules[i].x - RobotTargets.ghost_offset[0] for i in range(config.count)]
            angles = [math.atan2(y_comp[i], x_comp[i])+math.pi/2 for i in range(config.count)]

            dists = [math.pow(y_comp[i]**2 + x_comp[i]**2, 0.5) for i in range(config.count)]
            max_dist = max(dists)
            max_dist = 1 if max_dist < 1 else max_dist
            mods = [dists[i] / max_dist for i in range(config.count)]

            turns_y = [mods[i] * -turn * math.sin(angles[i]) for i in range(config.count)]
            turns_x = [mods[i] * -turn * math.cos(angles[i]) for i in range(config.count)]

            y = [drive + turns_y[i] for i in range(config.count)]
            x = [strafe + turns_x[i] for i in range(config.count)]

            RobotTargets.vels = [pow(y[i]**2 + x[i]**2, 0.5) for i in range(config.count)]
            new_rots = [(math.degrees(math.atan2(y[i], x[i]))-90 if (x != 0 and y != 0) else RobotTargets.rots[i]) for i in range(config.count)]
            for i in range(config.count):
                if abs(RobotTargets.vels[i]) < 0.01:
                    continue
                diff = RobotTargets.rots[i] - new_rots[i]
                if diff > 90:
                    RobotTargets.rots[i] = new_rots[i] + 180
                    RobotTargets.vels[i] *= -1
                elif diff < -90:
                    RobotTargets.rots[i] = new_rots[i] - 180
                    RobotTargets.vels[i] *= -1
                else:
                    RobotTargets.rots[i] = new_rots[i]
        elif RobotTargets.curr_state == 1:
            pass
        else:
            print('Robot currently in unsupported state! Reverting back to 0!')
            RobotTargets.curr_state = 0


BYTE = c_ubyte
XUSER_MAX_COUNT = 4
class XINPUT_BUTTONS(Structure):
	_fields_ = [
		("DPAD_UP", WORD, 1),
		("DPAD_DOWN", WORD, 1),
		("DPAD_LEFT", WORD, 1),
		("DPAD_RIGHT", WORD, 1),
		("START", WORD, 1),
		("BACK", WORD, 1),
		("LEFT_THUMB", WORD, 1),
		("RIGHT_THUMB", WORD, 1),
		("LEFT_SHOULDER", WORD, 1),
		("RIGHT_SHOULDER", WORD, 1),
		("_reserved_1_", WORD, 1),
		("_reserved_1_", WORD, 1),
		("A", WORD, 1),
		("B", WORD, 1),
		("X", WORD, 1),
		("Y", WORD, 1)
	]
	def __repr__(self):
		r = []
		for name, type, size in self._fields_:
			if "reserved" in name:
				continue
			r.append("{}={}".format(name, getattr(self, name)))
		args = ', '.join(r)
		return f"XINPUT_GAMEPAD({args})"
class XINPUT_GAMEPAD(Structure):
	"""Describes the current state of the Xbox 360 Controller.
	
	https://docs.microsoft.com/en-us/windows/win32/api/xinput/ns-xinput-xinput_gamepad
	
	wButtons is a bitfield describing currently pressed buttons
	"""
	_fields_ = [
		("wButtons", XINPUT_BUTTONS),
		("bLeftTrigger", BYTE),
		("bRightTrigger", BYTE),
		("sThumbLX", SHORT),
		("sThumbLY", SHORT),
		("sThumbRX", SHORT),
		("sThumbRY", SHORT),
	]
	def __repr__(self):
		r = []
		for name, type in self._fields_:
			r.append("{}={}".format(name, getattr(self, name)))
		args = ', '.join(r)
		return f"XINPUT_GAMEPAD({args})"
class XINPUT_STATE(Structure):
	"""Represents the state of a controller.
	
	https://docs.microsoft.com/en-us/windows/win32/api/xinput/ns-xinput-xinput_state
	
	dwPacketNumber: State packet number. The packet number indicates whether
		there have been any changes in the state of the controller. If the
		dwPacketNumber member is the same in sequentially returned XINPUT_STATE
		structures, the controller state has not changed.
	"""
	_fields_ = [
		("dwPacketNumber", DWORD),
		("Gamepad", XINPUT_GAMEPAD)
	]
	def __repr__(self):
		return f"XINPUT_STATE(dwPacketNumber={self.dwPacketNumber}, Gamepad={self.Gamepad})"
class XInput:
	"""Minimal XInput API wrapper"""
	def __init__(self):
		# https://docs.microsoft.com/en-us/windows/win32/xinput/xinput-versions
		# XInput 1.4 is available only on Windows 8+.
		# Older Windows versions are End Of Life anyway.
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

class RobotTargets():
    pow = 0.5
    ghost_offset = [0, 0]
    curr_state = 0
    state_count = 1
    state_names = [
        "Direct Drive",
        "GPS Drive",
    ]

    def state_name():
        return RobotTargets.state_names[RobotTargets.curr_state]
def sign_of(value):
    if value < 0:
        return -1
    return 1
def angle_between(angle1, angle2):
    diff = angle1 - angle2
    while diff > 180:
        diff -= 360
    while diff < -180:
        diff += 360
    return diff
def get_heading(pin):
    return Vector3(Matrix4x4.from_rotation(InertialMotionUnit(pin).rotation_axis())._backing.rotation.eulerAngles).y

def main_loop():
    controller.read_controls(config)
    config.apply_targets()

if __name__ == "__main__":
    config = RobotConfig(
        imu_pin=12,
        modules=[
            SwerveModule(x=-12, y=-4, drive_pin=0,  rot_pin=4,  imu_pin=8),  # BL
            SwerveModule(x=12,  y=-4, drive_pin=1,  rot_pin=5,  imu_pin=9),  # BR
            SwerveModule(x=-12, y=4,  drive_pin=2,  rot_pin=6,  imu_pin=10), # FL
            SwerveModule(x=12,  y=4,  drive_pin=3,  rot_pin=7,  imu_pin=11), # FR
            SwerveModule(x=0,   y=-7, drive_pin=13, rot_pin=14, imu_pin=15), # BC
            SwerveModule(x=0,   y=7,  drive_pin=16, rot_pin=17, imu_pin=18), # FC
        ]
    )
    RobotTargets.vels = [0 for _ in range(config.count)]
    RobotTargets.rots = [0 for _ in range(config.count)]
    controller = ControlScheme(
        gamepad_controls={
            "drive":       lambda x: x.sThumbLY / MAX_JOY_VAL,
            "strafe":      lambda x: x.sThumbLX / MAX_JOY_VAL,
            "turn":        lambda x: x.sThumbRX / MAX_JOY_VAL,
            "swap_modes":  lambda x: x.wButtons.BACK > 0,
            "ghost_up":    lambda x: x.wButtons.DPAD_UP > 0,
            "ghost_down":  lambda x: x.wButtons.DPAD_DOWN > 0,
            "ghost_left":  lambda x: x.wButtons.DPAD_LEFT > 0,
            "ghost_right": lambda x: x.wButtons.DPAD_RIGHT > 0,
            "speed_up":    lambda x: x.wButtons.LEFT_SHOULDER > 0,
            "speed_down":  lambda x: x.bLeftTrigger / MAX_TRIG_VAL > 0.7,
        },
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
    )
    while True:
        main_loop()

# POSSIBLE GAMEPAD BUTTONS / VALUES
# x.wButtons.DPAD_UP > 0
# x.wButtons.DPAD_DOWN > 0
# x.wButtons.DPAD_LEFT > 0
# x.wButtons.DPAD_RIGHT > 0
# x.wButtons.START > 0
# x.wButtons.BACK > 0
# x.wButtons.LEFT_THUMB > 0
# x.wButtons.RIGHT_THUMB > 0
# x.wButtons.LEFT_SHOULDER > 0
# x.wButtons.RIGHT_SHOULDER > 0
# x.wButtons.A > 0
# x.wButtons.B > 0
# x.wButtons.X > 0
# x.wButtons.Y > 0
# x.bLeftTrigger / MAX_TRIG_VAL
# x.bRightTrigger / MAX_TRIG_VAL
# x.sThumbLX / MAX_JOY_VAL
# x.sThumbLY / MAX_JOY_VAL
# x.sThumbRX / MAX_JOY_VAL
# x.sThumbRY / MAX_JOY_VAL