from common.op_params import opParams
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
    self.gone_fast_yet = False
    self.steer_rate_limited = False
    self.timer = 0
    self.steerErrorMod = False
    self.steer_type = int(0)
    self.on_timer = 0
    self.hightorqUnavailable = False
    self.acc_stop_timer = 0
    self.stop_button_spam = 0
    self.wheel_button_counter_prev = 0
    self.lead_dist_at_stop = 0

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
    self.hightorqUnavailable = False

    if Params().get_bool('LkasFullRangeAvailable'):
      wp_type = int(1)
    if Params().get_bool('ChryslerMangoMode'):
      wp_type = int(2)

    if enabled:
      if self.timer < 99 and wp_type == 1 and CS.out.vEgo < 65:
        self.timer += 1
      else:
        self.timer = 99
    else:
      self.timer = 0

    lkas_active = self.timer == 99

    # steer torque
    new_steer = int(round(actuators.steer * CarControllerParams.STEER_MAX))
    if not Params().get_bool('ChryslerMangoMode'):
      apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last,
                                                   CS.out.steeringTorqueEps, CarControllerParams)
    else:
      apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last,
                                                     CS.out.steeringTorqueEps/4., CarControllerParams) # WP multiply factor

    if not Params().get_bool('ChryslerMangoMode') and not Params().get_bool('LkasFullRangeAvailable'):
      moving_fast = CS.out.vEgo > CS.CP.minSteerSpeed  # for status message
      if CS.out.vEgo > (CS.CP.minSteerSpeed - 0.5):  # for command high bit
        self.gone_fast_yet = True
      elif self.car_fingerprint in (CAR.PACIFICA_2019_HYBRID, CAR.PACIFICA_2020, CAR.JEEP_CHEROKEE_2019):
        if CS.out.vEgo < (CS.CP.minSteerSpeed - 3.0):
          self.gone_fast_yet = False  # < 14.5m/s stock turns off this bit, but fine down to 13.5
      lkas_active = moving_fast and enabled

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
    
    if self.steer_type == int(0) and CS.out.gearShifter in (GearShifter.drive, GearShifter.low) and not CS.apaFault:
      self.hightorqUnavailable = True

    self.apaActive = CS.apasteerOn and self.steer_type == 2

    can_sends = []

    #*** control msgs ***

    self.resume_press = False
    if CS.acc_hold and CS.out.standstill:
      self.acc_stop_timer += 1
      if self.acc_stop_timer > 180: # send resume spam at 1.8 sec; looks like ACC auto resumes by itself if lead moves within 2 seconds
        self.resume_press = True
    else:
      self.acc_stop_timer = 0
      self.lead_dist_at_stop = CS.lead_dist

    if CS.acc_button_pressed:
      self.stop_button_spam = self.ccframe + 50 # stop spamming for 500msec if driver pressed ant acc steering wheel button

    wheel_button_counter_change = CS.wheel_button_counter != self.wheel_button_counter_prev
    if wheel_button_counter_change:
      self.wheel_button_counter_prev = CS.wheel_button_counter

    if (self.ccframe % 10 < 5) and wheel_button_counter_change and self.ccframe >= self.stop_button_spam:
      button_type = None
      if not enabled and pcm_cancel_cmd and CS.out.cruiseState.enabled:
        button_type = 'ACC_CANCEL'
      elif enabled and self.resume_press and CS.lead_dist > self.lead_dist_at_stop:
        button_type = 'ACC_RESUME'
      elif not CS.out.brakePressed and not CS.out.cruiseState.enabled and CS.out.cruiseState.available and opParams().get('brakereleaseAutoResume') and CS.out.gasPressed and CS.out.gearShifter == GearShifter.drive:
        button_type = 'ACC_RESUME'

      if button_type is not None:
        new_msg = create_wheel_buttons(self.packer, CS.wheel_button_counter + 1, button_type)
        can_sends.append(new_msg)

    # LKAS_HEARTBIT is forwarded by Panda so no need to send it here.
    # frame is 100Hz (0.01s period)
    if (self.ccframe % 2 == 0) and wp_type == 2:  # 0.02s period
      new_msg = create_apa_hud(
          self.packer, self.apaActive, CS.apaFault, lkas_active,
          self.steer_type)
      can_sends.append(new_msg)

    if (self.ccframe % 2 == 0) and wp_type != 2:  # 0.25s period
      new_msg = create_lkas_hud(
          self.packer, CS.out.gearShifter, lkas_active,
          self.hud_count, self.steer_type)
      can_sends.append(new_msg)

    if self.ccframe % 25 == 0:
      self.hud_count += 1

    new_msg = create_lkas_command(self.packer, int(apply_steer), lkas_active, frame)
    can_sends.append(new_msg)

    self.ccframe += 1
    self.prev_frame = frame

    return can_sends
