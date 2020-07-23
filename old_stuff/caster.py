from time import sleep, time
import odrive
from odrive.enums import *
import numpy as np

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