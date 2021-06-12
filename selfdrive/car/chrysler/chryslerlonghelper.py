from selfdrive.config import Conversions as CV
from common.numpy_fast import clip
from numpy import interp

SET_SPEED_MIN = 5 * CV.MPH_TO_MS
SET_SPEED_MAX = 120 * CV.MPH_TO_MS
LONG_PRESS_TIME = 50  # 500msec
SHORT_PRESS_STEP = 1
LONG_PRESS_STEP = 5
# Accel Hard limits
ACCEL_HYST_GAP = 0.0  # don't change accel command for small oscillations within this value
ACCEL_MAX = 2.  # m/s2
ACCEL_MIN = -3.8  # m/s2
ACCEL_SCALE = 1.

DEFAULT_DECEL = 4.0 # m/s2
START_BRAKE_THRESHOLD = -0.00001 # m/s2
STOP_BRAKE_THRESHOLD = 0.1 # m/s2
START_GAS_THRESHOLD = 0.00001 # m/s2
STOP_GAS_THRESHOLD = 0.0 # m/s2

CHIME_TIME = 8
CHIME_GAP_TIME = 5

def setspeedlogic(set_speed, acc_enabled, acc_enabled_prev, setplus, setminus, resbut, timer, ressetspeed, short_press, vego, gas_set, gas, gas_timer):

    set_speed = int(round((set_speed * CV.MS_TO_MPH), 0))
    vego = int(round((vego * CV.MS_TO_MPH), 0))

    if not acc_enabled and acc_enabled_prev:
      ressetspeed = set_speed

    if not gas or (gas_timer > 200 and setminus):
      gas_set = False
      gas_timer = 0
    elif gas_set:
      gas_timer += 1

    if acc_enabled_prev and acc_enabled:
      if setplus:
        if not short_press:
          if gas and not gas_set:
            if set_speed < vego:
              set_speed = vego
            else:
              set_speed += SHORT_PRESS_STEP
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
            set_speed = vego
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
    elif acc_enabled and not short_press:
      if resbut:
        set_speed = ressetspeed
      else:
        set_speed = vego
      short_press = True
      timer += 1
    else:
      short_press = False
      timer = 0

    set_speed = set_speed * CV.MPH_TO_MS
    set_speed = clip(set_speed, SET_SPEED_MIN, SET_SPEED_MAX)

    return set_speed, short_press, timer, gas_set, ressetspeed, gas_timer


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

def accel_rate_limit(accel_lim, prev_accel_lim, stopped):
 # acceleration jerk = 2.0 m/s/s/s
 # brake jerk = 3.8 m/s/s/s

  drBp = [   0., -0.15, -0.50,  -1.0,  -5.0]
  dra = [ 0.005, 0.007,  0.008, 0.01,  0.04]

  decel_rate = interp(accel_lim, drBp, dra)
  accel_rate = 0.005

  if not stopped:
    if accel_lim > 0:
      if accel_lim > prev_accel_lim:
        accel_lim = min(accel_lim, prev_accel_lim + accel_rate)
      else:
        accel_lim = max(accel_lim, prev_accel_lim - 0.02)
    else:
      if accel_lim < prev_accel_lim:
        accel_lim = max(accel_lim, prev_accel_lim - decel_rate)
      else:
        accel_lim = min(accel_lim, prev_accel_lim + accel_rate)

  return accel_lim


def cluster_chime(chime_val, enabled, enabled_prev, chime_timer, gap_timer):

  if not enabled_prev and enabled:
    chime_val = 4
    chime_timer = CHIME_TIME
    gap_timer = 0
  elif enabled_prev and not enabled:
    chime_val = 7
    chime_timer = CHIME_TIME
    gap_timer = CHIME_GAP_TIME

  if chime_timer > 0:
    chime_timer -= 1
  elif gap_timer > 0:
    gap_timer -= 1
    if gap_timer == 0:
      chime_timer = CHIME_TIME

  return chime_val, chime_timer, gap_timer
'''
def cal_curve_speed(self, sm, v_ego, frame):

  if frame % 10 == 0:
    md = sm['modelV2']
    if len(md.position.x) == TRAJECTORY_SIZE and len(md.position.y) == TRAJECTORY_SIZE:
      x = md.position.x
      y = md.position.y
      dy = np.gradient(y, x)
      d2y = np.gradient(dy, x)
      curv = d2y / (1 + dy ** 2) ** 1.5
      curv = curv[5:TRAJECTORY_SIZE - 10]
      a_y_max = 2.975 - v_ego * 0.0375  # ~1.85 @ 75mph, ~2.6 @ 25mph
      v_curvature = np.sqrt(a_y_max / np.clip(np.abs(curv), 1e-4, None))
      model_speed = np.mean(v_curvature) * 0.9 * self.curvature_gain

      if model_speed < v_ego:
        self.curve_speed_ms = float(max(model_speed, MIN_CURVE_SPEED))
      else:
        self.curve_speed_ms = 255.

      if np.isnan(self.curve_speed_ms):
        self.curve_speed_ms = 255.
    else:
      self.curve_speed_ms = 255.
'''
