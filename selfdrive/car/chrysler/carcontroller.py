from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, \
  create_wheel_buttons, create_apa_hud
from selfdrive.car.chrysler.values import CAR, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.car.interfaces import GearShifter
from common.params import Params

class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.ccframe = 0
    self.prev_frame = -1
    self.hud_count = 0
    self.car_fingerprint = CP.carFingerprint
#    self.gone_fast_yet = False
    self.steer_rate_limited = False
    self.timer = 0
    self.steerErrorMod = False
    self.steer_type = int(0)
    self.on_timer = 0

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, actuators, pcm_cancel_cmd, hud_alert):
    # this seems needed to avoid steering faults and to force the sync with the EPS counter
    frame = CS.lkas_counter
    if self.prev_frame == frame:
      return []

    # *** compute control surfaces ***
    if self.on_timer < 200 and CS.veh_on:
      self.on_timer += 1

    wp_type = int(0)

    if Params().get('LkasFullRangeAvailable') == b'1':
      wp_type = int(1)
    if Params().get('ChryslerMangoMode') == b'1':
      wp_type = int(2)

    if enabled:
      if CS.out.steeringAngleDeg > 50. and wp_type == 1 and self.timer > 97:
        self.timer = 97
      if self.timer < 99 and wp_type == 1 and CS.out.vEgo < 65:
        self.timer += 1
      else:
        self.timer = 99
    else:
      self.timer = 0

    lkas_active = self.timer == 99

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last,
                                                   CS.out.steeringTorqueEps, CarControllerParams)
    #self.steer_rate_limited = new_steer != apply_steer

    #moving_fast = CS.out.vEgo > CS.CP.minSteerSpeed  # for status message
    #if CS.out.vEgo > (CS.CP.minSteerSpeed - 0.5):  # for command high bit
    #  self.gone_fast_yet = True
    #elif self.car_fingerprint in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
    #  if CS.out.vEgo < (CS.CP.minSteerSpeed - 3.0):
    #    self.gone_fast_yet = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
    #lkas_active = moving_fast and enabled

    if not lkas_active:
      apply_steer = 0

    self.steer_rate_limited = new_steer != apply_steer

    self.apply_steer_last = apply_steer

    if CS.out.standstill:
      self.steer_type = wp_type

    if wp_type != 2:
      self.steerErrorMod = CS.steerError
      if self.steerErrorMod:
        self.steer_type = int(0)
    elif CS.apaFault or CS.out.gearShifter not in (GearShifter.drive, GearShifter.low) or \
            abs(CS.out.steeringAngleDeg) > 330. or self.on_timer < 200 or CS.apa_steer_status:
      self.steer_type = int(0)

    self.apaActive = CS.apasteerOn and self.steer_type == 2

    can_sends = []

    #*** control msgs ***

   # if pcm_cancel_cmd:
   #   # TODO: would be better to start from frame_2b3
   #   new_msg = create_wheel_buttons(self.packer, self.ccframe, cancel=True)
   #   can_sends.append(new_msg)

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 100Hz (0.01s period)
    if (self.ccframe % 2 == 0) and wp_type == 2:  # 0.02s period
      if (CS.lkas_car_model != -1):
        new_msg = create_apa_hud(
            self.packer, CS.out.gearShifter, self.apaActive, CS.apaFault, hud_alert, lkas_active,
            self.hud_count, CS.lkas_car_model, self.steer_type)
        can_sends.append(new_msg)
        self.hud_count += 1
    if (self.ccframe % 2 == 0) and wp_type != 2:  # 0.25s period
      if (CS.lkas_car_model != -1):
        new_msg = create_lkas_hud(
            self.packer, CS.out.gearShifter, lkas_active, hud_alert,
            self.hud_count, CS.lkas_car_model, self.steer_type)
        can_sends.append(new_msg)
        self.hud_count += 1
    #new_msg = create_lkas_command(self.packer, int(apply_steer), self.gone_fast_yet, frame)
    new_msg = create_lkas_command(self.packer, int(apply_steer), lkas_active, frame)
    can_sends.append(new_msg)

    self.ccframe += 1
    self.prev_frame = frame

    return can_sends
