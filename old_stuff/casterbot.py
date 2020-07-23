
from time import sleep, time
import odrive
from odrive.enums import *
import numpy as np
from controller1 import Controller
from inputs import get_gamepad, devices
import argparse

class Caster():
  def __init__(self, serial_number, wheel_axis_number):
    self.serial_number = serial_number
    self.wheel_axis_number = wheel_axis_number
    self.odrv = None
    self.wheel = None
    self.arm = None
    self.home = 0.0
    self.WHEEL_GEAR = 28
    self.ARM_GEAR = 30
  
  def connect(self):
    self.odrv = odrive.find_any(serial_number=self.serial_number)
    print("Connected to #%s = %d" % (self.serial_number, self.odrv.serial_number))
    if self.wheel_axis_number == 0:
      self.wheel = self.odrv.axis0
      self.arm = self.odrv.axis1
    else:
      self.wheel = self.odrv.axis1
      self.arm = self.odrv.axis0
    self.set_velocity_mode()

  def set_velocity_mode(self):
    self.wheel.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL
    self.arm.controller.config.control_mode = CTRL_MODE_VELOCITY_CONTROL

  def calibrate_wheel(self):
    self.wheel.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

  def calibrate_arm(self):
    self.arm.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

  def is_idle(self):
    return np.all([axis.current_state == AXIS_STATE_IDLE for axis in [self.wheel, self.arm]])

  def reset_errors(self):
    self.wheel.error = 0
    self.arm.error = 0

  def has_errors(self):
    if self.wheel.error != 0:
      print("%s has wheel error!" % self.serial_number)
      return True
    elif self.arm.error != 0:
      print("%s has arm error!" % self.serial_number)
      return True
    return False

  def set_closed_loop(self):
    self.wheel.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    self.arm.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

  def set_idle(self):
    self.wheel.requested_state = AXIS_STATE_IDLE
    self.arm.requested_state = AXIS_STATE_IDLE    

  def set_velocity(self, wheel_vel, arm_vel):
    w = (wheel_vel + arm_vel) / (2*np.pi) * self.wheel.encoder.config.cpr * self.WHEEL_GEAR
    a = arm_vel / (2*np.pi) * self.arm.encoder.config.cpr * self.ARM_GEAR
    self.wheel.controller.vel_setpoint = w * -1
    self.arm.controller.vel_setpoint = a * -1
  
  def set_home(self):
    self.wheel_home = self.wheel.encoder.pos_estimate
    self.arm_home = self.arm.encoder.pos_estimate

  def goto_home(self):
    self.set_velocity(0, 0)
    self.arm.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    self.arm.controller.pos_setpoint = self.arm_home

  def get_position(self):
    return np.array([
      (self.wheel.encoder.pos_estimate - self.wheel_home) / self.WHEEL_GEAR,
      (self.arm.encoder.pos_estimate - self.arm_home) / self.ARM_GEAR,
    ]) / self.wheel.encoder.config.cpr * 2*np.pi * -1

def all_is_idle(casters):
  return np.all([caster.is_idle() for caster in casters])

def wait_until_idle(casters):
  while not all_is_idle(casters):
    sleep(1)

def run_setup_sequence(casters, calibrate):
  print("Connecting!")
  for caster in casters:
    caster.connect()
    caster.reset_errors()
    caster.set_idle()

  if calibrate:
    print("Calibrate wheels")
    for caster in casters:
      caster.calibrate_wheel()

    wait_until_idle(casters)

    print("Calibrate arms")
    for caster in casters:
      caster.calibrate_arm()

    wait_until_idle(casters)

  print("Validate setup")
  for caster in casters:
    if caster.has_errors():
      exit()

  print("Set home")
  for caster in casters:
    caster.set_home()

  print("Set closed loop control")
  for caster in casters:
    caster.set_closed_loop()

  print("Setup complete!")

def run_shutdown_sequence(casters):
  print("Stop!")
  for caster in casters:
    caster.set_velocity(0, 0)
    caster.set_idle()

def normalize_joystick(val):
  val = np.clip(val / 32000, -1, 1)
  if np.abs(val) < 0.15:
    val = 0.0
  return val

def main():

  parser = argparse.ArgumentParser(description='Run casterbot.')
  parser.add_argument('--calib', action='store_true', help='Run full encoder calibration')
  args = parser.parse_args()

  casters = [
    Caster("208C3373304B", 0),
    Caster("206A337F304B", 1),
    Caster("206A339F304B", 0),
    Caster("20773398304B", 1),
  ]

  controller = Controller(
      dt=0.01,
      wheel_offset=0.08,
      arm_length=0.415/2,
      radius=0.075/2,
      n_casters=4,
      mass=0.8,
      inertia=0.025,
      #beta=[0., np.pi/2, np.pi, 3*np.pi/2]
      beta=[0., np.pi/2, np.pi, -np.pi/2]
  )

  run_setup_sequence(casters, args.calib)

  running = True

  # Desired speed
  x = np.array([0.0, 0.0, 0.0])
  xx = np.array([0.0, 0.0, 0.0])
  MAX_LINEAR = 0.1
  MAX_ANGULAR = 0.2

  while True:
    
    shutdown = False

    events = devices.gamepads[0]._do_iter()
    if events:
      for event in events:
        if event.code == "ABS_X":
          xx[1] = normalize_joystick(event.state) * -1 * MAX_LINEAR
        elif event.code == "ABS_Y":
          xx[0] = normalize_joystick(event.state) * -1 * MAX_LINEAR
        elif event.code == "ABS_RX":
          xx[2] = normalize_joystick(event.state) * -1 * MAX_ANGULAR
        elif event.code == "BTN_EAST" and event.state == 0:
          # Release button -> B
          shutdown = True

    x += 0.1 * (xx - x)

    if shutdown:
      break
    
    q = []
    for caster in casters:
      wheel_pos, arm_pos = caster.get_position()
      q += [arm_pos, wheel_pos]

    u = controller.control(np.copy(q), np.copy(x))
    
    for i, caster in enumerate(casters):
      arm_vel, wheel_vel = u[i*2:i*2+2]
      caster.set_velocity(wheel_vel, arm_vel)
  
  run_shutdown_sequence(casters)

  print("Bye!")

if __name__ == "__main__":
  main()