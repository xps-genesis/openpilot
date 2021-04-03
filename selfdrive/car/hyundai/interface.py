#!/usr/bin/env python3
from cereal import car
from common.op_params import opParams
from common.params import Params
from selfdrive.config import Conversions as CV
from selfdrive.car.hyundai.values import CAR, Buttons
from selfdrive.car import STD_CARGO_KG, scale_rot_inertia, scale_tire_stiffness, gen_empty_fingerprint
from selfdrive.car.interfaces import CarInterfaceBase

EventName = car.CarEvent.EventName
ButtonType = car.CarState.ButtonEvent.Type

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)
    self.buttonEvents = []
    self.cp2 = self.CS.get_can2_parser(CP)
    self.visiononlyWarning = False
    self.belowspeeddingtimer = 0.
    self.enabled_prev = False

  @staticmethod
  def compute_gb(accel, speed):
    return float(accel) / 1.0

  @staticmethod
  def get_params(candidate, fingerprint=gen_empty_fingerprint(), car_fw=[]):  # pylint: disable=dangerous-default-value
    ret = CarInterfaceBase.get_std_params(candidate, fingerprint)

    ret.carName = "hyundai"
    ret.safetyModel = car.CarParams.SafetyModel.hyundai

    # Most Hyundai car ports are community features for now
    ret.communityFeature = candidate not in [CAR.SONATA, CAR.PALISADE]

    ret.steerActuatorDelay = 0.3  # Default delay
    ret.steerRateCost = 0.5
    ret.steerLimitTimer = 0.1
    tire_stiffness_factor = 0.7
    ret.maxSteeringAngleDeg = 90.
    ret.startAccel = 1.0

    eps_modified = False
    for fw in car_fw:
      if fw.ecu == "eps" and b"," in fw.fwVersion:
        eps_modified = True


    #Long tuning Params -  make individual params for cars, baseline Hyundai genesis
    ret.longitudinalTuning.kpBP = [0., .3, 10., 35.]
    ret.longitudinalTuning.kpV = [1.8, .8, .3, .3]
    ret.longitudinalTuning.kiBP = [0., .3, 15., 35.]
    ret.longitudinalTuning.kiV = [0.15, .055, .05, .045]
    ret.longitudinalTuning.deadzoneBP = [0., .5]
    ret.longitudinalTuning.deadzoneV = [0.00, 0.00]
    ret.gasMaxBP = [0., 1., 1.1, 15., 40.]
    ret.gasMaxV = [2., 2., 2., 1.68, 1.3]
    ret.brakeMaxBP = [0., 5., 5.1]
    ret.brakeMaxV = [3.5, 3.5, 3.5]  # safety limits to stop unintended deceleration
    ret.longitudinalTuning.kfBP = [0., 5., 10., 20., 30.]
    ret.longitudinalTuning.kfV = [1., 1., 1., .75, .5]

    ret.lateralTuning.pid.kpBP = [0., 10., 30.]
    ret.lateralTuning.pid.kpV = [0.01, 0.02, 0.03]
    ret.lateralTuning.pid.kiBP = [0., 10., 30.]
    ret.lateralTuning.pid.kiV = [0.001, 0.0015, 0.002]
    ret.lateralTuning.pid.kfBP = [0., 10., 30.]
    ret.lateralTuning.pid.kfV = [0.000015, 0.00002, 0.000025]

    if opParams().get('Enable_INDI'):
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [0.]
      ret.lateralTuning.indi.innerLoopGainV = [2.]
      ret.lateralTuning.indi.outerLoopGainBP = [0.]
      ret.lateralTuning.indi.outerLoopGainV = [3.0]
      ret.lateralTuning.indi.timeConstantBP = [0.]
      ret.lateralTuning.indi.timeConstantV = [1.4]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [2.]

    if candidate in [CAR.SANTA_FE, CAR.SANTA_FE_2017]:
      ret.mass = 3982. * CV.LB_TO_KG + STD_CARGO_KG
      ret.wheelbase = 2.766
      ret.steerRatio = 16.55
    elif candidate in [CAR.SONATA, CAR.SONATA_HEV]:
      ret.mass = 1513. + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.27 * 1.15
    elif candidate in [CAR.SONATA_2019, CAR.SONATA_HEV_2019]:
      ret.mass = 4497. * CV.LB_TO_KG
      ret.wheelbase = 2.804
      ret.steerRatio = 13.27 * 1.15
    elif candidate == CAR.PALISADE:
      ret.mass = 1999. + STD_CARGO_KG
      ret.wheelbase = 2.90
      ret.steerRatio = 13.75 * 1.15
      if eps_modified:
        ret.maxSteeringAngleDeg = 1000.
    elif candidate == CAR.KIA_SORENTO:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.1
    elif candidate in [CAR.ELANTRA, CAR.ELANTRA_GT_I30]:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4
      ret.minSteerSpeed = 32 * CV.MPH_TO_MS
    elif candidate == CAR.ELANTRA_2020:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 15.4
    elif candidate == CAR.HYUNDAI_GENESIS:
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
      ret.minSteerSpeed = 55 * CV.KPH_TO_MS
    elif candidate == CAR.GENESIS_G70:
      ret.lateralTuning.init('indi')
      ret.lateralTuning.indi.innerLoopGainBP = [0.]
      ret.lateralTuning.indi.innerLoopGainV = [3.5]
      ret.lateralTuning.indi.outerLoopGainBP = [0.]
      ret.lateralTuning.indi.outerLoopGainV = [2.0]
      ret.lateralTuning.indi.timeConstantBP = [0.]
      ret.lateralTuning.indi.timeConstantV = [1.4]
      ret.lateralTuning.indi.actuatorEffectivenessBP = [0.]
      ret.lateralTuning.indi.actuatorEffectivenessV = [2.3]

      ret.mass = 1640.0 + STD_CARGO_KG
      ret.wheelbase = 2.84
      ret.steerRatio = 13.56
    elif candidate == CAR.GENESIS_G80:
      ret.steerActuatorDelay = 0.4
      ret.steerRateCost = 0.45
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate == CAR.GENESIS_G90:
      ret.mass = 2060. + STD_CARGO_KG
      ret.wheelbase = 3.01
      ret.steerRatio = 16.5
    elif candidate in [CAR.KIA_OPTIMA, CAR.KIA_OPTIMA_HEV]:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
    elif candidate == CAR.KIA_STINGER:
      ret.mass = 1825. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15
    elif candidate == CAR.KONA:
      ret.mass = 1275. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15
    elif candidate in [CAR.KONA_HEV, CAR.KONA_EV]:
      ret.mass = 1685. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15
    elif candidate in [CAR.IONIQ_HEV, CAR.IONIQ_EV_LTD]:
      ret.mass = 1490. + STD_CARGO_KG
      ret.wheelbase = 2.7
      ret.steerRatio = 13.73 * 1.15
    elif candidate == CAR.KIA_FORTE:
      ret.mass = 3558. * CV.LB_TO_KG
      ret.wheelbase = 2.80
      ret.steerRatio = 13.75 * 1.15
    elif candidate == CAR.KIA_CEED:
      ret.mass = 1350. + STD_CARGO_KG
      ret.wheelbase = 2.65
      ret.steerRatio = 13.75 * 1.15
    elif candidate == CAR.KIA_SPORTAGE:
      ret.mass = 1985. + STD_CARGO_KG
      ret.wheelbase = 2.78
      ret.steerRatio = 14.4 * 1.15
    elif candidate == CAR.VELOSTER:
      ret.steerActuatorDelay = 0.1  # Default delay
      ret.steerRateCost = 0.5
      ret.steerLimitTimer = 0.4
      ret.mass = 3558. * CV.LB_TO_KG # actual: 2987
      ret.wheelbase = 2.80 # actual: 2.649
      ret.steerRatio = 13.75 * 1.15
      ret.lateralTuning.pid.kpBP = [0., 10., 30.]
      ret.lateralTuning.pid.kpV = [0.25, 0.25, 0.25]
      ret.lateralTuning.pid.kiBP = [0., 10., 30.]
      ret.lateralTuning.pid.kiV = [0.05, 0.05, 0.05]
      ret.lateralTuning.pid.kfBP = [0., 10., 30.]
      ret.lateralTuning.pid.kfV = [0.00005, 0.00005, 0.00005]
    elif candidate in [CAR.KIA_NIRO_HEV, CAR.KIA_NIRO_EV]:
      ret.steerRatio = 13.73
      ret.mass = 1737. + STD_CARGO_KG
      ret.wheelbase = 2.7
    elif candidate in [CAR.GRANDEUR, CAR.GRANDEUR_HEV]:
      ret.mass = 1719. + STD_CARGO_KG
      ret.wheelbase = 2.8
      ret.steerRatio = 12.5* 1.15
    elif candidate in [CAR.KIA_CADENZA, CAR.KIA_CADENZA_HEV]:
      ret.mass = 1575. + STD_CARGO_KG
      ret.wheelbase = 2.85
      ret.steerRatio = 12.5 * 1.15
    elif candidate == CAR.KIA_SELTOS:
      ret.mass = 1310. + STD_CARGO_KG
      ret.wheelbase = 2.6
      ret.steerRatio = 13.73  # Spec

    # these cars require a special panda safety mode due to missing counters and checksums in the messages

    ret.mdpsHarness = Params().get('MdpsHarnessEnabled') == b'1'
    ret.sasBus = 0 if (688 in fingerprint[0] or not ret.mdpsHarness) else 1
    ret.fcaBus = 0 if 909 in fingerprint[0] else 2 if 909 in fingerprint[2] else -1
    ret.bsmAvailable = True if 1419 in fingerprint[0] else False
    ret.lfaAvailable = True if 1157 in fingerprint[2] else False
    ret.lvrAvailable = True if 871 in fingerprint[0] else False
    ret.evgearAvailable = True if 882 in fingerprint[0] else False
    ret.emsAvailable = True if 608 and 809 in fingerprint[0] else False

    if Params().get('SccEnabled') == b'1':
      ret.sccBus = 2 if 1057 in fingerprint[2] and Params().get('SccHarnessPresent') == b'1' else 0 if 1057 in fingerprint[0] else -1
    else:
      ret.sccBus = -1

    ret.radarOffCan = (ret.sccBus == -1)

    ret.openpilotLongitudinalControl = Params().get('LongControlEnabled') == b'1' and not (ret.sccBus == 0)

    if ret.openpilotLongitudinalControl:
      ret.radarTimeStep = .05

    if candidate in [ CAR.HYUNDAI_GENESIS, CAR.IONIQ_EV_LTD, CAR.IONIQ_HEV, CAR.KONA_EV, CAR.KIA_NIRO_EV, CAR.KIA_SORENTO, CAR.SONATA_2019,
                      CAR.KIA_OPTIMA, CAR.VELOSTER, CAR.KIA_STINGER, CAR.GENESIS_G70, CAR.SONATA_HEV, CAR.SANTA_FE, CAR.GENESIS_G80,
                      CAR.GENESIS_G90, CAR.KIA_CADENZA, CAR.KIA_CADENZA_HEV]:
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiLegacy

    if ret.mdpsHarness or \
            (candidate in [CAR.KIA_OPTIMA_HEV, CAR.SONATA_HEV, CAR.IONIQ_HEV, CAR.SONATA_HEV_2019,
                          CAR.KIA_CADENZA_HEV, CAR.GRANDEUR_HEV, CAR.KIA_NIRO_HEV, CAR.KONA_HEV]):
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunity

    if ret.radarOffCan or (ret.sccBus == 2) or Params().get('EnableOPwithCC') == b'0':
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunityNonscc

    if ret.mdpsHarness or opParams().get('smartMDPS'):
      ret.minSteerSpeed = 0.

    ret.centerToFront = ret.wheelbase * 0.4

    # TODO: get actual value, for now starting with reasonable value for
    # civic and scaling by mass and wheelbase
    ret.rotationalInertia = scale_rot_inertia(ret.mass, ret.wheelbase)

    # TODO: start from empirically derived lateral slip stiffness for the civic and scale by
    # mass and CG position, so all cars will have approximately similar dyn behaviors
    ret.tireStiffnessFront, ret.tireStiffnessRear = scale_tire_stiffness(ret.mass, ret.wheelbase, ret.centerToFront,
                                                                         tire_stiffness_factor=tire_stiffness_factor)

    ret.enableCamera = True

    ret.radarDisablePossible = Params().get('RadarDisableEnabled') == b'1'

    ret.enableCruise = Params().get('EnableOPwithCC') == b'1' and ret.sccBus == 0

    if ret.radarDisablePossible:
      ret.openpilotLongitudinalControl = True
      ret.safetyModel = car.CarParams.SafetyModel.hyundaiCommunityNonscc # todo based on toggle
      ret.sccBus = -1
      ret.enableCruise = False
      ret.radarOffCan = True
      if ret.fcaBus == 0:
        ret.fcaBus = -1

    return ret

  def update(self, c, can_strings):
    self.cp.update_strings(can_strings)
    self.cp2.update_strings(can_strings)
    self.cp_cam.update_strings(can_strings)

    ret = self.CS.update(self.cp, self.cp2, self.cp_cam)
    ret.canValid = self.cp.can_valid and self.cp2.can_valid and self.cp_cam.can_valid

    # speeds
    ret.steeringRateLimited = self.CC.steer_rate_limited if self.CC is not None else False

    events = self.create_common_events(ret)

    # low speed steer alert hysteresis logic (only for cars with steer cut off above 10 m/s)
    if ret.vEgo < (self.CP.minSteerSpeed + .56) and self.CP.minSteerSpeed > 10. and self.CC.enabled:
      if not self.low_speed_alert and self.belowspeeddingtimer < 100:
        events.add(car.CarEvent.EventName.belowSteerSpeedDing)
        self.belowspeeddingtimer +=1
      else:
        self.belowspeeddingtimer = 0.
        self.low_speed_alert = True
    if ret.vEgo > (self.CP.minSteerSpeed + .84) or not self.CC.enabled:
      self.low_speed_alert = False
      self.belowspeeddingtimer = 0.
    if self.low_speed_alert:
      events.add(car.CarEvent.EventName.belowSteerSpeed)

    if self.CP.sccBus == 2:
      self.CP.enableCruise = self.CC.usestockscc

    if self.enabled_prev and not self.CC.enabled and not self.CP.enableCruise:
      ret.cruiseState.enabled = False
    self.enabled_prev = self.CC.enabled

    if self.CS.brakeHold and not self.CC.usestockscc:
      events.add(EventName.brakeHold)
    if self.CS.parkBrake and not self.CC.usestockscc:
      events.add(EventName.parkBrake)
    if self.CS.brakeUnavailable and not self.CC.usestockscc:
      events.add(EventName.brakeUnavailable)
    if not self.visiononlyWarning and self.CP.radarDisablePossible and self.CC.enabled and not self.low_speed_alert:
      events.add(EventName.visiononlyWarning)
      self.visiononlyWarning = True

    buttonEvents = []
    if self.CS.cruise_buttons != self.CS.prev_cruise_buttons:
      be = car.CarState.ButtonEvent.new_message()
      be.pressed = self.CS.cruise_buttons != 0 
      but = self.CS.cruise_buttons
      if but == Buttons.RES_ACCEL:
        be.type = ButtonType.accelCruise
      elif but == Buttons.SET_DECEL:
        be.type = ButtonType.decelCruise
      elif but == Buttons.GAP_DIST:
        be.type = ButtonType.gapAdjustCruise
      elif but == Buttons.CANCEL:
        be.type = ButtonType.cancel
      else:
        be.type = ButtonType.unknown
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    if self.CS.cruise_main_button != self.CS.prev_cruise_main_button:
      be = car.CarState.ButtonEvent.new_message()
      be.type = ButtonType.altButton3
      be.pressed = bool(self.CS.cruise_main_button)
      buttonEvents.append(be)
      self.buttonEvents = buttonEvents

    ret.buttonEvents = self.buttonEvents

    # handle button press
    if not self.CP.enableCruise:
      for b in self.buttonEvents:
        if b.type == ButtonType.decelCruise and b.pressed \
                and (not ret.brakePressed or ret.standstill):
          events.add(EventName.buttonEnable)
          events.add(EventName.pcmEnable)
        if b.type == ButtonType.accelCruise and b.pressed \
                and ((self.CC.setspeed > self.CC.clu11_speed - 2) or ret.standstill or self.CC.usestockscc):
          events.add(EventName.buttonEnable)
          events.add(EventName.pcmEnable)
        if b.type == ButtonType.cancel and b.pressed or self.CS.lkasbutton and opParams().get('enableLKASbutton'):
          events.add(EventName.buttonCancel)
          events.add(EventName.pcmDisable)
        if b.type == ButtonType.altButton3 and b.pressed:
          events.add(EventName.buttonCancel)
          events.add(EventName.pcmDisable)

    ret.events = events.to_msg()

    self.CS.out = ret.as_reader()
    return self.CS.out

  def apply(self, c):
    can_sends = self.CC.update(c.enabled, self.CS, self.frame, c.actuators,
                               c.cruiseControl.cancel, c.hudControl.visualAlert, c.hudControl.leftLaneVisible,
                               c.hudControl.rightLaneVisible, c.hudControl.leftLaneDepart, c.hudControl.rightLaneDepart,
                               c.hudControl.setSpeed, c.hudControl.leadVisible, c.hudControl.leadDistance,
                               c.hudControl.leadvRel, c.hudControl.leadyRel)
    self.frame += 1
    return can_sends
