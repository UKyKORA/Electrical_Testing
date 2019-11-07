import time
import math
import odrive
from odrive.enums import *

class ODriveTester:
    def __init__(self):
        '''find and configure motor'''
        print("finding an odrive...")
        self.test_drive = odrive.find_any()

        print("starting calibration...")
        self.test_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.test_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

         # this says that it will hold the motor at a given position
         # self.test_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    def set_pos(self, pos):
        '''set pos of the current found motor'''
        self.test_drive.axis0.controller.pos_setpoint = pos

    def set_vel(self, vel):
        self.test_drive.axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        if vel > self.test_drive.axis.controller.config.vel_limit:
            print("Velocity given greater than velocity limit. Please adjust"
                  "velocity limit or lower velocity")
        else:
            self.test_drive.axis.controller.vel_setpoint = vel

    def get_current_and_voltage_readings(self):
        '''gets IV readings from important areas'''
        bus_voltage = self.test_drive.vbus_voltage
        gpio_voltages = [None]*4
        for i in [1, 2, 3, 4]:
            gpio_voltages[i-1] = self.test_drive.get_adc_voltage(i)
        return {
            'bus_voltage' : bus_voltage,
            'gpio_voltages' : gpio_voltages
        }

    def motor_config(self, attr, setting):
        '''configure motor settings'''
        try:
            setattr(self.test_drive.axis0.motor, attr, setting)
        except AttributeError as err:
            print(f'Failed setting with attribute error: {str(err)}')
            return False
        except ValueError as err:
            print(f'invalid value for {attr}: {setting}')
            return False
        return True

    def get_current_config(self):
        '''list out major motor parameters'''
        ctrl_cfg = self.test_drive.controller.config
        motor_cfg = self.test_drive.motor.config
        return {
            'controller_config' : ctrl_cfg,
            'motor_config' : motor_cfg
        }

    def save_config(self):
        self.test_drive.save_configuration()

    def clear_all_settings(self):
        '''
        meant to clear all current positions and velocities so
        that it wont bug with future controls
        '''
        return None

    def do_sine_wave_for(self, endtime):
        '''do sine wave for an amount of seconds just for fun'''
        nowtime_mono = time.monotonic()
        nowtime = time.time()
        while time.time() - nowtime != endtime:
            setpoint = 10000.0 * math.sin((time.monotonic() - nowtime_mono)*2)
            print("goto " + str(int(setpoint)))
            self.test_drive.axis0.controller.pos_setpoint = setpoint
            time.sleep(0.01)

    def reboot(self):
        '''reboots odrive'''
        self.test_drive.reboot()

def start_odrive_repl():
    '''
    starts a repl for controlling the odrive,
    also should post IV and parameters asynchronously
    '''

    # TODO: have a thread that posts current config and 
    # readings to a window

    # Use the terminal for repl
    my_odrive = ODriveTester()
    not_done = True
    while not_done:
        # expect method then atrributes seperated by spaces
        control = input('Control: ')
        method_and_params = control.split(' ')
        if method_and_params[0] == 'exit':
            not_done = False
            continue
        try:
            # calls the method with its params
            getattr(my_odrive, method_and_params[0])(*method_and_params[1:])
        except AttributeError:
            print('Incorrect Control')

if __name__ == '__main__':
    start_odrive_repl()
