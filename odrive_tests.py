'''
ODrive Testing and Control Module

This is some testing code for the Odrive motors,
hopefully capable of being useful for later code as well.

Useful links:
https://github.com/madcowswe/ODrive/blob/master/docs/commands.md
https://github.com/madcowswe/ODrive/blob/master/docs/odrivetool.md
https://github.com/madcowswe/ODrive/blob/master/docs/testing.md

TODOs:
* Use liveplotter to plot the position/current/velocity etc. (find on link 2)
* Configure tests within yaml/json files and load yaml/json
* Setup better for ros control.
'''
import time
import math
import multiprocessing
import tkinter as tk
import odrive
# from odrive.enums import *
from odrive.enums import (AXIS_STATE_FULL_CALIBRATION_SEQUENCE,
                          AXIS_STATE_IDLE,
                          CTRL_MODE_VELOCITY_CONTROL,
                          CTRL_MODE_POSITION_CONTROL,
                          CTRL_MODE_CURRENT_CONTROL)

class ODriveMotor:
    '''
    Testings apparatus for Odrive motors.
    This is structured to hopefully be useful to the final odrive code as well
    '''
    def __init__(self):
        '''find and configure motor'''
        print("finding an odrive...")
        self.test_drive = odrive.find_any()
        self.is_on = True

        print("starting calibration...")
        self.test_drive.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        while self.test_drive.axis0.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

         # this says that it will hold the motor at a given position
         # self.test_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    def set_pos(self, pos):
        '''set pos of the current found motor'''
        self.test_drive.axis0.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
        self.test_drive.axis0.controller.pos_setpoint = pos

    def set_vel(self, vel):
        '''sets velocity of the motor'''
        self.test_drive.axis.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
        if vel > self.test_drive.axis.controller.config.vel_limit:
            print("Velocity given greater than velocity limit. Please adjust"
                  "velocity limit or lower velocity")
        else:
            self.test_drive.axis.controller.vel_setpoint = vel

    def set_current(self, curr):
        '''sets current of the motor'''
        self.test_drive.axis.controller.config.control_mode = CTRL_MODE_CURRENT_CONTROL
        if curr > self.test_drive.axis.controller.config.current_limit:
            print("Current given is greater than the current limit in the config. "
                  "Please adjust the config and try again")
        else:
            self.test_drive.axis.controller.current_setpoint = curr


    def get_current_and_voltage_readings(self):
        '''gets IV readings from important areas'''
        bus_voltage = self.test_drive.vbus_voltage
        gpio_voltages = [None]*4
        for i in [1, 2, 3, 4]:
            gpio_voltages[i-1] = self.test_drive.get_adc_voltage(i)

        encoder_pos = self.test_drive.axis0.encoder.pos_estimate
        encoder_vel = self.test_drive.axis0.encoder.vel_estimate
        return {
            'bus_voltage' : bus_voltage,
            'gpio_voltages' : gpio_voltages,
            'encoder_pos' : encoder_pos,
            'encoder_vel' : encoder_vel
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
        ctrl_cfg = self.test_drive.axis0.controller.config
        motor_cfg = self.test_drive.axis0.motor.config
        axis_state = self.test_drive.axis0.current_state
        return {
            'controller_config' : ctrl_cfg,
            'motor_config' : motor_cfg,
            'axis_state' : axis_state
        }

    def save_config(self):
        '''saves this configuration across boots of the microcontroller'''
        self.test_drive.save_configuration()

    def load_yaml_conf(self):
        '''loads in a configuration from a yaml spec'''
        #TODO

    def clear_conf(self):
        '''clears configuration'''
        self.test_drive.erase_configuration()

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

def start_odrive_repl(my_odrive):
    '''
    starts a repl for controlling the odrive,
    also should post IV and parameters asynchronously
    '''
    # Use the terminal for repl
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

def start_odrive_poster(my_odrive):
    '''updates gui with motor information every second'''
    win = tk.Tk()
    win.title("Odrive Sensors and Settings")
    win.geometry('800x400')
    win.configure(background='#CD5C5C')

    # configuration values
    conf_vals = tk.StringVar(value="waiting")
    current_conf_values = tk.Label(win, textvariable=conf_vals)
    current_conf_values.place(x=10, y=10)

    # current and voltage readings
    curr_vals = tk.StringVar(value="waiting")
    current_curr_values = tk.Label(win, textvariable=conf_vals)
    current_curr_values.place(x=10, y=200)

    while my_odrive.is_on:
        conf_dict = my_odrive.get_current_config()
        conf_vals.set(f'{conf_dict}')
        curr_dict = my_odrive.get_current_and_voltage_readings()
        curr_vals.set(f'{curr_dict}')
        win.update()
        time.sleep(0.5)

def start_odrive_testing():
    '''spins off repl from sensor posting'''
    my_odrive = ODriveMotor()
    procs = []
    procs.append(multiprocessing.Process(target=start_odrive_repl, args=(my_odrive)))
    procs.append(multiprocessing.Process(target=start_odrive_poster, args=(my_odrive)))
    for proc in procs:
        proc.join()

if __name__ == '__main__':
    start_odrive_testing()
