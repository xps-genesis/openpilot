from cereal import car
from selfdrive.car import make_can_msg


GearShifter = car.CarState.GearShifter
VisualAlert = car.CarControl.HUDControl.VisualAlert

def create_apa_hud(packer, apa_active, apa_fault, enabled, steer_type):
  # LKAS_HUD 0x2a6 (678) Controls what lane-keeping icon is displayed.

  color = 1  # default values are for park or neutral in 2017 are 0 0, but trying 1 1 for 2019
  lines = 1

  # had color = 1 and lines = 1 but trying 2017 hybrid style for now.
  if enabled and apa_active:
      color = 2  # control active, display green.
  if apa_fault:
    color = 3
  values = {
    "LKAS_ICON_COLOR": color,  # byte 0, last 2 bits
    "LKAS_LANE_LINES": lines,  # byte 2, last 4 bits
    "STEER_TYPE": steer_type,
    }
  return packer.make_can_msg("LKAS_HUD", 0, values)  # 0x2a6

def create_lkas_hud(packer, gear, lkas_active, hud_count, steer_type):
  # LKAS_HUD 0x2a6 (678) Controls what lane-keeping icon is displayed.

  color = 1  # default values are for park or neutral in 2017 are 0 0, but trying 1 1 for 2019
  lines = 1
  alerts = 0

  if hud_count < (1 * 4):  # first 3 seconds, 4Hz
    alerts = 1
  # had color = 1 and lines = 1 but trying 2017 hybrid style for now.
  if gear in (GearShifter.drive, GearShifter.reverse, GearShifter.low):
    if lkas_active:
      color = 2  # control active, display green.
      lines = 6
    else:
      color = 1  # control off, display white.
      lines = 1

  values = {
    "LKAS_ICON_COLOR": color,  # byte 0, last 2 bits
    "LKAS_LANE_LINES": lines,  # byte 2, last 4 bits
    "LKAS_ALERTS": alerts,  # byte 3, last 4 bits
    "STEER_TYPE": steer_type,
    }

  return packer.make_can_msg("LKAS_HUD", 0, values)  # 0x2a6


def create_lkas_command(packer, apply_steer, lkas_active, frame):
  # LKAS_COMMAND 0x292 (658) Lane-keeping signal to turn the wheel.
  values = {
    "LKAS_STEERING_TORQUE": apply_steer,
    "LKAS_HIGH_TORQUE": lkas_active,
    "COUNTER": frame % 0x10,
  }
  return packer.make_can_msg("LKAS_COMMAND", 0, values)


def create_wheel_buttons(packer, counter, button_type):
  # WHEEL_BUTTONS (571) Message sent to cancel ACC.
  values = {
    button_type: 1,
    "COUNTER": counter
  }
  return packer.make_can_msg("WHEEL_BUTTONS", 0, values)
