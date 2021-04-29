from common.op_params import opParams
from selfdrive.car import apply_toyota_steer_torque_limits
from selfdrive.car.chrysler.chryslercan import create_lkas_hud, create_lkas_command, \
  create_wheel_buttons, create_apa_hud, create_op_acc_1, create_op_acc_2, create_op_dashboard, create_op_chime
from selfdrive.car.chrysler.values import CAR, CarControllerParams
from opendbc.can.packer import CANPacker
from selfdrive.car.interfaces import GearShifter
from common.params import Params
from selfdrive.config import Conversions as CV
from common.numpy_fast import clip

SET_SPEED_MIN = 5 * CV.MPH_TO_MS
LONG_PRESS_TIME = 50  # 500msec
SHORT_PRESS_STEP = 1
LONG_PRESS_STEP = 5
# Accel Hard limits
ACCEL_HYST_GAP = 0.01  # don't change accel command for small oscillations within this value
ACCEL_MAX = 1.  # m/s2
ACCEL_MIN = -3.8  # m/s2
ACCEL_SCALE = 1.

DEFAULT_DECEL = 4.0 # m/s2
START_BRAKE_THRESHOLD = -0.160 # m/s2
STOP_BRAKE_THRESHOLD = 0.001 # m/s2
START_GAS_THRESHOLD = 0.002 # m/s2
STOP_GAS_THRESHOLD = -0.159 # m/s2

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
    #OPLong starts here
    self.op_long_enable = CP.openpilotLongitudinalControl
    self.acc_available = True
    self.acc_enabled = False
    self.set_speed = SET_SPEED_MIN
    self.set_speed_timer = 0
    self.short_press = True
    self.gas_press_set_speed = False
    self.cruise_state = 0
    self.cruise_icon = 0
    self.acc_pre_brake = False
    self.accel_lim_prev = 0.
    self.accel_lim = 0.
    self.accel_steady = 0.
    self.accel_active = False
    self.decel_active = False
    self.chime = 0
    self.chime_timer = 0
    self.enabled_prev = False
    self.play_times = 0

    self.packer = CANPacker(dbc_name)

  def update(self, enabled, CS, actuators, pcm_cancel_cmd, hud_alert, op_lead_rvel,
             op_set_speed, op_lead_visible, op_lead_dist, long_stopping, long_starting):
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
    if Params().get_bool('ChryslerMangoLat'):
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
    apply_steer = apply_toyota_steer_torque_limits(new_steer, self.apply_steer_last,
                                                   CS.out.steeringTorqueEps, CarControllerParams)

    if not Params().get_bool('ChryslerMangoLat') and not Params().get_bool('LkasFullRangeAvailable'):
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
      self.stop_button_spam = self.ccframe + 50 # stop spamming for 500msec if driver pressed any acc steering wheel button

    wheel_button_counter_change = CS.wheel_button_counter != self.wheel_button_counter_prev
    if wheel_button_counter_change:
      self.wheel_button_counter_prev = CS.wheel_button_counter

    self.op_cancel_cmd = False

    if not self.op_long_enable and (self.ccframe % 10 < 5) and wheel_button_counter_change and self.ccframe >= self.stop_button_spam:
      button_type = None
      if not enabled and pcm_cancel_cmd and CS.out.cruiseState.enabled:
        button_type = 'ACC_CANCEL'
        self.op_cancel_cmd = True
      elif enabled and self.resume_press and (CS.lead_dist > self.lead_dist_at_stop or op_lead_rvel > 0 or 15 > CS.lead_dist >= 6.):
        button_type = 'ACC_RESUME'
      elif not CS.out.brakePressed and not CS.out.cruiseState.enabled and \
              CS.out.cruiseState.available and opParams().get('brakereleaseAutoResume') and \
              CS.out.gasPressed and CS.out.gearShifter == GearShifter.drive:
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


    #############################################################################
    # Chrysler OP long- Recreate ACC ECU here                                   #
    #############################################################################

    # build ACC enabling logic
    ####################################################################################################################
    if CS.acc_on_button and not CS.acc_on_button_prev:
       self.acc_available = not self.acc_available

    if not self.acc_enabled and not CS.out.brakePressed and self.acc_available and \
            (CS.acc_setplus_button or CS.acc_setminus_button or CS.acc_resume_button):
      self.acc_enabled = True
    elif  self.acc_enabled and not self.acc_available or CS.acc_cancel_button or pcm_cancel_cmd:
      self.acc_enabled = False

    self.set_speed, self.short_press, self.set_speed_timer, self.gas_press_set_speed = setspeedlogic(self.set_speed, self.acc_enabled,
                                                                         CS.acc_setplus_button, CS.acc_setminus_button,
                                                                         self.set_speed_timer, SET_SPEED_MIN,
                                                                         self.short_press, CS.out.vEgoRaw, self.gas_press_set_speed, CS.out.gasPressed)

    self.cruise_state, self.cruise_icon = cruiseiconlogic(self.acc_enabled, self.acc_available, op_lead_visible)

    # Build ACC long control signals
    ####################################################################################################################
    self.stop_req = enabled and CS.out.standstill and not CS.out.gasPressed

    # gas and brake
    self.accel_lim_prev = self.accel_lim
    apply_accel = actuators.gas - actuators.brake

    apply_accel, self.accel_steady = accel_hysteresis(apply_accel, self.accel_steady)
    apply_accel = clip(apply_accel * ACCEL_SCALE, ACCEL_MIN, ACCEL_MAX)

    self.accel_lim = apply_accel
    apply_accel = accel_rate_limit(self.accel_lim, self.accel_lim_prev)

    self.decel_val = DEFAULT_DECEL
    self.trq_val = STOP_GAS_THRESHOLD * CV.ACCEL_TO_NM

    if not CS.out.gasPressed and CS.acc_override and\
            (apply_accel <= START_BRAKE_THRESHOLD or self.decel_active and apply_accel <= STOP_BRAKE_THRESHOLD):
      self.decel_active = True
      self.decel_val = apply_accel
    else:
      self.decel_active = False
      
    self.go_req = False

    if not CS.out.brakePressed and (apply_accel >= START_GAS_THRESHOLD or self.accel_active and apply_accel >= STOP_GAS_THRESHOLD):
      self.accel_active = True
      self.trq_val = max(apply_accel * CV.ACCEL_TO_NM, CS.axle_torq + 100)
      self.stop_req = False
      self.go_req = CS.out.standstill
    else:
      self.accel_active = False

    self.chime , self.chime_timer = cluster_chime(self.chime, enabled, self.enabled_prev, self.chime_timer, self.play_times)
    self.enabled_prev = enabled
      # Senf ACC msgs on can
    ####################################################################################################################
    if self.ccframe % 2 == 0:
      new_msg = create_op_acc_1(self.packer, self.accel_active, self.trq_val)
      can_sends.append(new_msg)
      new_msg = create_op_acc_2(self.packer, self.acc_available, self.acc_enabled, self.stop_req, self.go_req, self.acc_pre_brake, self.decel_val, self.decel_active)
      can_sends.append(new_msg)
    if self.ccframe % 6 == 0:
      new_msg = create_op_dashboard(self.packer, self.set_speed, self.cruise_state, self.cruise_icon, op_lead_visible, op_lead_dist, self.op_long_enable)
      can_sends.append(new_msg)

    new_msg = create_op_chime(self.packer, self.chime, self.chime_timer)
    can_sends.append(new_msg)

    self.ccframe += 1
    self.prev_frame = frame

    return can_sends


def setspeedlogic(set_speed, acc_enabled, setplus, setminus, timer, set_speed_min, short_press, vego, gas_set, gas):

    set_speed = int(round((set_speed * CV.MS_TO_MPH), 0))
    set_speed_min = int(round((set_speed_min * CV.MS_TO_MPH),0))
    vego = int(round((vego * CV.MS_TO_MPH), 0))

    if acc_enabled:
      if setplus:
        if not short_press:
          if gas and not gas_set:
            set_speed = max(vego, set_speed_min)
            gas_set = True
          else:
            set_speed += SHORT_PRESS_STEP
          short_press = True
        elif timer % LONG_PRESS_TIME == 0:
            set_speed += (LONG_PRESS_STEP - set_speed % LONG_PRESS_STEP)
        timer += 1
      elif setminus:
        if not short_press:
          if gas and not gas_set:
            set_speed = max(vego, set_speed_min)
            gas_set = True
          else:
            set_speed -= SHORT_PRESS_STEP
          short_press = True
        elif timer % LONG_PRESS_TIME == 0:
          if set_speed % LONG_PRESS_STEP > 0:
            set_speed += (LONG_PRESS_STEP - set_speed % LONG_PRESS_STEP)
          set_speed -= LONG_PRESS_STEP
        timer += 1
      else:
        timer = 0
        short_press = False
    else:
      set_speed = max(vego, set_speed_min)
      short_press = False

    if not gas:
      gas_set = False

    set_speed = max(set_speed, set_speed_min) * CV.MPH_TO_MS

    return set_speed, short_press, timer, gas_set


def cruiseiconlogic(acc_enabled, acc_available, has_lead):
    if acc_enabled:
      cruise_state = 4  # ACC engaged
      if has_lead:
        cruise_icon = 15  # ACC green icon with 4 bar distance and lead
      else:
        cruise_icon = 11  # ACC green icon with 4 bar distance and no lead
    else:
      if acc_available:
        cruise_state = 3 # ACC on
        cruise_icon = 5 # ACC white icon with 4 bar distance
      else:
        cruise_state = 0
        cruise_icon = 0

    return cruise_state, cruise_icon

def accel_hysteresis(accel, accel_steady):

  # for small accel oscillations within ACCEL_HYST_GAP, don't change the accel command
  if accel > accel_steady + ACCEL_HYST_GAP:
    accel_steady = accel - ACCEL_HYST_GAP
  elif accel < accel_steady - ACCEL_HYST_GAP:
    accel_steady = accel + ACCEL_HYST_GAP
  accel = accel_steady

  return accel, accel_steady

def accel_rate_limit(accel_lim, prev_accel_lim):
 # acceleration jerk = 2.0 m/s/s/s
 # brake jerk = 3.8 m/s/s/s
  if accel_lim > 0:
    if accel_lim > prev_accel_lim:
      accel_lim = min(accel_lim, prev_accel_lim + 0.02)
    else:
      accel_lim = max(accel_lim, prev_accel_lim - 0.038)
  else:
    if accel_lim < prev_accel_lim:
      accel_lim = max(accel_lim, prev_accel_lim - 0.038)
    else:
      accel_lim = min(accel_lim, prev_accel_lim + 0.01)

  return accel_lim


def cluster_chime(chime, enabled, enabled_prev, chime_timer, play_times):

  if not enabled_prev and enabled:
    chime = 4
    chime_timer = 0
    play_times = 1
  elif enabled_prev and not enabled:
    chime = 5
    chime_timer = 0
    play_times = 2

  if play_times > 0 and chime_timer == 0:
    play_times -= 1

  if chime_timer < 21:
    chime_timer += 1
  elif chime_timer == 21 and play_times > 0:
    chime_timer = 0

  return chime, chime_timer


