import numpy as np
from common.numpy_fast import clip, interp
from common.op_params import opParams

GainSaS_BP = [0., 3.9, 4., 5., 10., 20., 40.]
Gain_g = [0.05, .065, .085, .1, .12, .14, .16]

GainV_BP = [0., 20., 20.01, 30.]
Gain_V = [0.6, .85, 1.05, 1.2]

def apply_deadzone(error, deadzone):
  if error > deadzone:
    error -= deadzone
  elif error < - deadzone:
    error += deadzone
  else:
    error = 0.
  return error


class PIController:
  def __init__(self, k_p, k_i, k_f, k_d, pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_f = k_f  # feedforward gain
    self._k_d = k_d  # feedforward gain

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.i_rate = 1.0 / rate
    self.sat_limit = sat_limit
    self.convert = convert

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_f(self):
    return interp(self.speed, self._k_f[0], self._k_f[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.i = 0.0
    self.f = 0.0
    self.d = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.errors = []

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0., freeze_integrator=False):
    self.speed = speed

    if opParams().get('nonlinearsas'):
      self.nl_p = interp(abs(setpoint), GainSaS_BP, Gain_g) * interp(self.speed, GainV_BP, Gain_V)
    else:
      self.nl_p = 0.

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * (self.k_p + self.nl_p)
    self.f = feedforward * self.k_f

    self.d = 0.
    if len(self.errors) >= 5:  # makes sure list is long enough
      self.d = (error - self.errors[-5]) / 5  # get deriv in terms of 100hz (tune scale doesn't change)
      self.d *= self.k_d

    if override:
      self.i -= self.i_unwind_rate * float(np.sign(self.i))
    else:
      i = self.i + error * self.k_i * self.i_rate
      control = self.p + self.f + i

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.i = i

    control = self.p + self.f + self.i + self.d
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    self.saturated = self._check_saturation(control, check_saturation, error)

    self.errors.append(float(error))
    while len(self.errors) > 5:
      self.errors.pop(0)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control


class PIDController:
  def __init__(self, k_p, k_i, k_f, k_d, pos_limit=None, neg_limit=None, rate=100, sat_limit=0.8, convert=None):
    self.enable_long_derivative = False
    self._k_p = k_p  # proportional gain
    self._k_i = k_i  # integral gain
    self._k_d = k_d  # derivative gain
    self._k_f = k_f  # feedforward gain

    self.max_accel_d = 0.22352  # 0.5 mph/s

    self.pos_limit = pos_limit
    self.neg_limit = neg_limit

    self.sat_count_rate = 1.0 / rate
    self.i_unwind_rate = 0.3 / rate
    self.rate = 1.0 / rate
    self.sat_limit = sat_limit
    self.convert = convert
    self.last_kf = 0.
    self.last_error = 0.
    self.last_setpoint = 0.
    self.last_vlead = 0.

    self.reset()

  @property
  def k_p(self):
    return interp(self.speed, self._k_p[0], self._k_p[1])

  @property
  def k_i(self):
    return interp(self.speed, self._k_i[0], self._k_i[1])

  @property
  def k_d(self):
    return interp(self.speed, self._k_d[0], self._k_d[1])

  @property
  def k_f(self):
    return interp(self.speed, self._k_f[0], self._k_f[1])

  def _check_saturation(self, control, check_saturation, error):
    saturated = (control < self.neg_limit) or (control > self.pos_limit)

    if saturated and check_saturation and abs(error) > 0.1:
      self.sat_count += self.sat_count_rate
    else:
      self.sat_count -= self.sat_count_rate

    self.sat_count = clip(self.sat_count, 0.0, 1.0)

    return self.sat_count > self.sat_limit

  def reset(self):
    self.p = 0.0
    self.id = 0.0
    self.f = 0.0
    self.sat_count = 0.0
    self.saturated = False
    self.control = 0
    self.last_setpoint = 0.0
    self.last_error = 0.0
    self.last_kf = 0.0
    self.hasreset = True
    self.atargetfuture = 0
    self.locktarget = False
    self.last_vlead = 0.

  def update(self, setpoint, measurement, speed=0.0, check_saturation=True, override=False, feedforward=0., deadzone=0.,
             freeze_integrator=False, leadvisible=False, leaddistance=0 , leadvel=0):
    self.speed = speed
    self.f = feedforward * self.k_f

    setpoint = 0.989 * setpoint # for cluster display difference

    error = float(apply_deadzone(setpoint - measurement, deadzone))
    self.p = error * self.k_p

    if override:
      self.id -= self.i_unwind_rate * float(np.sign(self.id))
    else:
      i = self.id + error * self.k_i * self.rate
      if self.last_error > 1.8 >= error and i > 0:
        i = 0
      i = min(i, 0.2)
      control = self.p + self.f + i

      if self.convert is not None:
        control = self.convert(control, speed=self.speed)

      # Update when changing i will move the control away from the limits
      # or when i will move towards the sign of the error
      if ((error >= 0 and (control <= self.pos_limit or i < 0.0)) or
          (error <= 0 and (control >= self.neg_limit or i > 0.0))) and \
         not freeze_integrator:
        self.id = i

    if self.enable_long_derivative:
      if abs(setpoint - self.last_setpoint) / self.rate < self.max_accel_d:  # if setpoint isn't changing much
        d = self.k_d * (error - self.last_error)
        if (self.id > 0 and self.id + d >= 0) or (self.id < 0 and self.id + d <= 0):  # if changing integral doesn't make it cross zero
          self.id += d

    control = self.p + self.f + self.id
    if self.convert is not None:
      control = self.convert(control, speed=self.speed)

    self.saturated = self._check_saturation(control, check_saturation, error)

    self.last_setpoint = float(setpoint)
    self.last_error = float(error)
    self.last_kf = float(self.f)

    self.control = clip(control, self.neg_limit, self.pos_limit)
    return self.control
