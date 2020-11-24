from numpy import clip

from cereal import car, messaging
from selfdrive.car import apply_std_steer_torque_limits
from selfdrive.car.hyundai.carstate import GearShifter
from selfdrive.car.hyundai.hyundaican import create_lkas11, create_clu11, create_lfa_mfa, \
                                             create_scc11, create_scc12, create_scc13, create_scc14, \
                                             create_scc42a, create_scc7d0, create_fca11, create_fca12, create_mdps12
from selfdrive.car.hyundai.values import Buttons, SteerLimitParams, CAR, FEATURES
from opendbc.can.packer import CANPacker
from selfdrive.config import Conversions as CV
from selfdrive.controls.lib.longcontrol import LongCtrlState

VisualAlert = car.CarControl.HUDControl.VisualAlert

# Accel Hard limits
ACCEL_HYST_GAP = 0.1  # don't change accel command for small oscillations within this value
ACCEL_MAX = 2.  # 2.0 m/s2
ACCEL_MIN = -3.5  # 3.5   m/s2
ACCEL_SCALE = 1.

def accel_hysteresis(accel, accel_steady):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady

def accel_rate_limit(accel_lim, prev_accel_lim):

  if accel_lim > 0:
    if accel_lim > prev_accel_lim:
      accel_lim = min(accel_lim, prev_accel_lim + 0.02)
    else:
      accel_lim = max(accel_lim, prev_accel_lim - 0.035)
  else:
    if accel_lim < prev_accel_lim:
      accel_lim = max(accel_lim, prev_accel_lim - 0.035)
    else:
      accel_lim = min(accel_lim, prev_accel_lim + 0.01)

  return accel_lim

def process_hud_alert(enabled, fingerprint, visual_alert, left_lane,
                      right_lane, left_lane_depart, right_lane_depart):

  sys_warning = (visual_alert == VisualAlert.steerRequired)
  if sys_warning:
      sys_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 3

  if enabled or sys_warning:
      sys_state = 3
  else:
      sys_state = 1

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if left_lane_depart:
    left_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2
  if right_lane_depart:
    right_lane_warning = 1 if fingerprint in [CAR.HYUNDAI_GENESIS, CAR.GENESIS_G90, CAR.GENESIS_G80] else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


class CarController():
  def __init__(self, dbc_name, CP, VM):
    self.apply_steer_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.cp_oplongcontrol = CP.openpilotLongitudinalControl
    self.packer = CANPacker(dbc_name)
    self.accel_steady = 0
    self.accel_lim_prev = 0.
    self.accel_lim = 0.
    self.steer_rate_limited = False
    self.p = SteerLimitParams(CP)
    self.usestockscc = True
    self.lead_visible = False
    self.lead_debounce = 0
    self.gapsettingdance = 2
    self.gapcount = 0
    self.current_veh_speed = 0
    self.lfainFingerprint = CP.lfaAvailable
    self.vdiff = 0
    self.resumebuttoncnt = 0
    self.lastresumeframe = 0
    self.fca11supcnt = self.fca11inc = self.fca11alivecnt = self.fca11cnt13 = self.scc11cnt = self.scc12cnt = 0
    self.counter_init = False
    self.fca11maxcnt = 0xD
    self.radarDisableActivated = False
    self.radarDisableResetTimer = 0
    self.radarDisableOverlapTimer = 0
    self.sendaccmode = not CP.radarDisablePossible
    self.enabled = False
    self.sm = messaging.SubMaster(['controlsState'])

  def update(self, enabled, CS, frame, actuators, pcm_cancel_cmd, visual_alert,
             left_lane, right_lane, left_lane_depart, right_lane_depart,
             set_speed, lead_visible, lead_dist, lead_vrel, lead_yrel):

    self.enabled = enabled
    # gas and brake
    self.accel_lim_prev = self.accel_lim
    apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    self.accel_lim = apply_accel
    apply_accel = accel_rate_limit(self.accel_lim, self.accel_lim_prev)

    # Steering Torque
    new_steer = actuators.steer * self.p.STEER_MAX
    apply_steer = apply_std_steer_torque_limits(new_steer, self.apply_steer_last, CS.out.steeringTorque, self.p)
    self.steer_rate_limited = new_steer != apply_steer

    # disable if steer angle reach 90 deg, otherwise mdps fault in some models
    self.high_steer_allowed = True if self.car_fingerprint in FEATURES["allow_high_steer"] else False
    lkas_active = enabled and ((abs(CS.out.steeringAngle) < 90.) or self.high_steer_allowed)

    # fix for Genesis hard fault at low speed
    if CS.out.vEgo < 55 * CV.KPH_TO_MS and self.car_fingerprint == CAR.HYUNDAI_GENESIS and CS.CP.minSteerSpeed > 0.:
      lkas_active = False

    if not lkas_active:
      apply_steer = 0

    if CS.CP.radarOffCan:
      self.usestockscc = not self.cp_oplongcontrol
    elif (CS.cancel_button_count == 3) and self.cp_oplongcontrol:
      self.usestockscc = not self.usestockscc

    if not self.usestockscc:
      self.gapcount += 1
      if self.gapcount == 50 and self.gapsettingdance == 2:
        self.gapsettingdance = 1
        self.gapcount = 0
      elif self.gapcount == 50 and self.gapsettingdance == 1:
        self.gapsettingdance = 4
        self.gapcount = 0
      elif self.gapcount == 50 and self.gapsettingdance == 4:
        self.gapsettingdance = 3
        self.gapcount = 0
      elif self.gapcount == 50 and self.gapsettingdance == 3:
        self.gapsettingdance = 2
        self.gapcount = 0

    self.apply_steer_last = apply_steer

    sys_warning, sys_state, left_lane_warning, right_lane_warning =\
      process_hud_alert(enabled, self.car_fingerprint, visual_alert,
                        left_lane, right_lane, left_lane_depart, right_lane_depart)

    speed_conv = CV.MS_TO_MPH if CS.is_set_speed_in_mph else CV.MS_TO_KPH

    self.clu11_speed = CS.clu11["CF_Clu_Vanz"]

    enabled_speed = 38 if CS.is_set_speed_in_mph else 60

    if self.clu11_speed > enabled_speed or not lkas_active or CS.out.gearShifter != GearShifter.drive:
      enabled_speed = self.clu11_speed

    self.current_veh_speed = int(CS.out.vEgo * speed_conv)

    self.clu11_cnt = frame % 0x10

    can_sends = []

    self.lfa_available = True if self.lfainFingerprint or self.car_fingerprint in FEATURES["send_lfa_mfa"] else False

    can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available, 0))

    if CS.CP.mdpsHarness:  # send lkas11 bus 1 if mdps
      can_sends.append(create_lkas11(self.packer, frame, self.car_fingerprint, apply_steer, lkas_active,
                                   CS.lkas11, sys_warning, sys_state, enabled,
                                   left_lane, right_lane,
                                   left_lane_warning, right_lane_warning, self.lfa_available, 1))

      can_sends.append(create_clu11(self.packer, 1, CS.clu11, Buttons.NONE, enabled_speed, self.clu11_cnt))

    if pcm_cancel_cmd and CS.scc12["ACCMode"] != 0 and not CS.out.standstill and CS.CP.enableCruise:
      self.vdiff = 0.
      self.resumebuttoncnt = 0
      can_sends.append(create_clu11(self.packer, CS.CP.sccBus, CS.clu11, Buttons.CANCEL, self.current_veh_speed, self.clu11_cnt))
    elif CS.out.cruiseState.standstill and CS.scc12["ACCMode"] != 0 and CS.vrelative > 0:
      self.vdiff += (CS.vrelative - self.vdiff)
      if (frame - self.lastresumeframe > 10) and (self.vdiff > .5 or CS.lead_distance > 6.):
        can_sends.append(create_clu11(self.packer, CS.CP.sccBus, CS.clu11, Buttons.RES_ACCEL, self.current_veh_speed, self.resumebuttoncnt))
        self.resumebuttoncnt += 1
        if self.resumebuttoncnt > 5:
          self.lastresumeframe = frame
          self.resumebuttoncnt = 0
    else:
      self.vdiff = 0.
      self.resumebuttoncnt = 0

    if CS.out.vEgo < 2.:
      self.sm.update(0)
      long_control_state = self.sm['controlsState'].longControlState
      self.acc_standstill = True if long_control_state == LongCtrlState.stopping else False
    else:
      self.acc_standstill = False

    if lead_visible:
      self.lead_visible = True
      self.lead_debounce = 50
    elif self.lead_debounce > 0:
      self.lead_debounce -= 1
    else:
      self.lead_visible = lead_visible

    self.setspeed = set_speed * speed_conv

    if enabled:
      self.sendaccmode = enabled

    if CS.CP.radarDisablePossible:
      self.radarDisableOverlapTimer += 1
      self.radarDisableResetTimer = 0
      if self.radarDisableOverlapTimer >= 30:
        self.radarDisableActivated = True
        if 200 > self.radarDisableOverlapTimer > 36:
          if frame % 41 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x02\x10\x03\x00\x00\x00\x00\x00'))
          elif frame % 43 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x03\x28\x03\x01\x00\x00\x00\x00'))
          elif frame % 19 == 0 or self.radarDisableOverlapTimer == 37:
            can_sends.append(create_scc7d0(b'\x02\x10\x85\x00\x00\x00\x00\x00'))  # this disables RADAR for
      else:
        self.counter_init = False
        can_sends.append(create_scc7d0(b'\x02\x10\x90\x00\x00\x00\x00\x00'))  # this enables RADAR
        can_sends.append(create_scc7d0(b'\x03\x29\x03\x01\x00\x00\x00\x00'))
    elif self.radarDisableActivated:
      can_sends.append(create_scc7d0(b'\x02\x10\x90\x00\x00\x00\x00\x00'))  # this enables RADAR
      can_sends.append(create_scc7d0(b'\x03\x29\x03\x01\x00\x00\x00\x00'))
      self.radarDisableOverlapTimer = 0
      if frame % 50 == 0:
        self.radarDisableResetTimer += 1
        if self.radarDisableResetTimer > 2:
          self.radarDisableActivated = False
          self.counter_init = True
    else:
      self.radarDisableOverlapTimer = 0
      self.radarDisableResetTimer = 0

    if (frame % 50 == 0 or self.radarDisableOverlapTimer == 37) and \
            CS.CP.radarDisablePossible and self.radarDisableOverlapTimer >= 30:
      can_sends.append(create_scc7d0(b'\x02\x3E\x00\x00\x00\x00\x00\x00'))

    if self.lead_visible:
      self.objdiststat = 1 if lead_dist < 25 else 2 if lead_dist < 40 else \
                         3 if lead_dist < 60 else 4 if lead_dist < 80 else 5
    else:
      self.objdiststat = 0

    # send scc to car if longcontrol enabled and SCC not on bus 0 or ont live
    if (CS.CP.sccBus == 2 or not self.usestockscc or self.radarDisableActivated) and self.counter_init:
      if frame % 2 == 0:
        self.scc12cnt += 1
        self.scc12cnt %= 0xF
        self.scc11cnt += 1
        self.scc11cnt %= 0x10
        self.fca11supcnt += 1
        self.fca11supcnt %= 0xF

        if self.fca11alivecnt == 1:
          self.fca11inc = 0
          if self.fca11cnt13 == 3:
            self.fca11maxcnt = 0x9
            self.fca11cnt13 = 0
          else:
            self.fca11maxcnt = 0xD
            self.fca11cnt13 += 1
        else:
          self.fca11inc += 4

        self.fca11alivecnt = self.fca11maxcnt - self.fca11inc

        can_sends.append(create_scc11(self.packer, enabled,
                                      self.setspeed, self.lead_visible, lead_dist, lead_vrel, lead_yrel,
                                      self.gapsettingdance,
                                      CS.out.standstill, CS.scc11,
                                      self.usestockscc, CS.CP.radarOffCan, self.scc11cnt, self.sendaccmode))

        can_sends.append(create_scc12(self.packer, apply_accel, enabled,
                                      self.acc_standstill, CS.out.gasPressed, CS.out.brakePressed,
                                      CS.out.stockAeb,
                                      CS.scc12, self.usestockscc, CS.CP.radarOffCan, self.scc12cnt))

        can_sends.append(create_scc14(self.packer, enabled, self.usestockscc, CS.out.stockAeb, apply_accel,
                                      CS.scc14, self.objdiststat, CS.out.gasPressed, self.acc_standstill, CS.out.vEgo))
        if CS.CP.fcaBus == -1:
          can_sends.append(create_fca11(self.packer, CS.fca11, self.fca11alivecnt, self.fca11supcnt))

      if frame % 20 == 0:
        can_sends.append(create_scc13(self.packer, CS.scc13))
        if CS.CP.fcaBus == -1:
          can_sends.append(create_fca12(self.packer))
      if frame % 50 == 0:
        can_sends.append(create_scc42a(self.packer))
    else:
      self.counter_init = True
      self.scc12cnt = CS.scc12init["CR_VSM_Alive"]
      self.scc11cnt = CS.scc11init["AliveCounterACC"]
      self.fca11alivecnt = CS.fca11init["CR_FCA_Alive"]
      self.fca11supcnt = CS.fca11init["Supplemental_Counter"]

    # 20 Hz LFA MFA message
    if frame % 5 == 0 and self.lfa_available:
      can_sends.append(create_lfa_mfa(self.packer, frame, enabled))

    can_sends.append(create_mdps12(self.packer, frame, CS.mdps12))

    return can_sends
