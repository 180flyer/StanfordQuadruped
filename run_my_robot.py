import numpy as np
import time
from src.Controller import Controller
from src.State import State
#  from pupper.HardwareInterface import HardwareInterface
from src.Command import Command
from pupper.Config import Configuration
from pupper.Kinematics import four_legs_inverse_kinematics
import curses
from curses import wrapper

def main(stdscr):
    """Main program
    """
    # Init Keyboard
    stdscr.clear()  # clear the screen
    curses.noecho()  # don't echo characters
    curses.cbreak()  # don't wait on CR
    stdscr.keypad(True)  # Map arrow keys to UpArrow, etc
    stdscr.nodelay(True)  # Don't block - do not wait on keypress

    # Create config
    config = Configuration()
    #  hardware_interface = HardwareInterface()
    command = Command()

    # Create controller and user input handles
    controller = Controller(
        config,
        four_legs_inverse_kinematics,
    )
    state = State()

    last_loop = time.time()

    print("Summary of gait parameters:")
    print("overlap time: ", config.overlap_time)
    print("swing time: ", config.swing_time)
    print("z clearance: ", config.z_clearance)
    print("x shift: ", config.x_shift)
    print("Waiting for L1 to activate robot.")

    key_press = ''
    while True:
        if key_press == 'q':
            break

        # Wait until the activate button has been pressed
        while True:
            try:
                key_press = stdscr.getkey()
            except:
                key_press = ''
            if key_press == 'q':
                break
            elif key_press == 'a':
                # command.activate_event = True
                # command.yaw_rate = 0
                # print(state.behavior_state.name)
                # print("\r")
                print("Robot Activated\r")
                break
        if key_press == 'q':
            break
        do_print_cmd = False
        while True:
            x_speed = command.horizontal_velocity[0]
            y_speed = command.horizontal_velocity[1]
            yaw_speed = command.yaw_rate
            new_x_speed = x_speed
            new_y_speed = y_speed
            new_yaw_speed = yaw_speed

            try:
                key_press = stdscr.getkey()
            except:
                key_press = ''

            if key_press == 'q':
                break
            elif key_press == 'a':
                # command.activate_event = True
                # controller.run(state, command)
                # command.activate_event = False
                # command.trot_event = False
                print("Robot Deactivate\r")
                break
            elif key_press == 't':
                command.trot_event = True
                print("Trot Event\r")
                do_print_cmd = True
            elif key_press == ' ':
                new_x_speed = 0
                new_y_speed = 0
                do_print_cmd = True
            elif key_press == 'k':
                new_yaw_speed = 0
                do_print_cmd = True
            elif key_press == 'i':
                new_x_speed = min(config.max_x_velocity, x_speed + config.max_x_velocity / 5.0)
                do_print_cmd = True
            elif key_press == ',':
                new_x_speed = max(-1 * config.max_x_velocity, x_speed - config.max_x_velocity / 5.0)
                do_print_cmd = True
            elif key_press == 'f':
                new_y_speed = max(-1 * config.max_y_velocity, y_speed - config.max_y_velocity / 5.0)
                do_print_cmd = True
            elif key_press == 's':
                new_y_speed = min(config.max_y_velocity, y_speed + config.max_y_velocity / 5.0)
                do_print_cmd = True
            elif key_press == 'd':
                new_y_speed = 0
                do_print_cmd = True
            elif key_press == 'j':
                new_yaw_speed = min(config.max_yaw_rate, yaw_speed + config.max_yaw_rate / 5.0)
                do_print_cmd = True
            elif key_press == 'l':
                new_yaw_speed = max(-1 * config.max_yaw_rate, yaw_speed - config.max_yaw_rate / 5.0)
                do_print_cmd = True

            command.horizontal_velocity = np.array([new_x_speed, new_y_speed])
            command.yaw_rate = new_yaw_speed

            now = time.time()
            if now - last_loop < config.dt:
                continue
            last_loop = time.time()

            # Read imu data. Orientation will be None if no data was available
            quat_orientation = (
                np.array([1, 0, 0, 0])
            )
            state.quat_orientation = quat_orientation

            # Step the controller forward by dt
            controller.run(state, command)
            if do_print_cmd:
                print_cmd(command)
                print_state(state)
                do_print_cmd = False

            # print(state.behavior_state.name + "\r")
            # print_state(state)

            # Update the pwm widths going to the servos
            # hardware_interface.set_actuator_postions(state.joint_angles)

            command.activate_event = False
            command.trot_event = False

def print_cmd(command):
    print("Cmd: Vx, Vy, Yaw: %0.2f, %0.2f, %0.2f\r" % (command.horizontal_velocity[0], command.horizontal_velocity[1], command.yaw_rate))
    print("Cmd: Height, Pitch, Roll, Activation: %0.2f, %0.2f, %0.2f, %d\r" % (command.height, command.pitch, command.roll, command.activation))

def print_state(state):
    #  print("State: Vx, Vy, Yaw: %0.2f, %0.2f, %0.2f\r" % (state.horizontal_velocity[0], state.horizontal_velocity[1], state.yaw_rate))
    #  print("State: Height, Pitch, Roll, Behavioral State: %0.2f, %0.2f, %0.2f, %s\r" % (state.height, state.pitch, state.roll, state.behavior_state.name))
    print("B.State: %s\r" % (state.behavior_state.name))
wrapper(main)
