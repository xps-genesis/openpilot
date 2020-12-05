import crcmod

from common.op_params import opParams
from selfdrive.car.hyundai.values import CAR, CHECKSUM

hyundai_checksum = crcmod.mkCrcFun(0x11D, initCrc=0xFD, rev=False, xorOut=0xdf)


def create_lkas11(packer, frame, car_fingerprint, apply_steer, steer_req,
                  lkas11, sys_warning, sys_state, enabled,
                  left_lane, right_lane,
                  left_lane_depart, right_lane_depart, lfa_available, bus):
  values = lkas11
  values["CF_Lkas_LdwsSysState"] = 3 if steer_req else sys_state
  values["CF_Lkas_SysWarning"] = sys_warning
  values["CF_Lkas_LdwsLHWarning"] = left_lane_depart
  values["CF_Lkas_LdwsRHWarning"] = right_lane_depart
  values["CR_Lkas_StrToqReq"] = apply_steer
  values["CF_Lkas_ActToi"] = steer_req
  values["CF_Lkas_ToiFlt"] = 0
  values["CF_Lkas_MsgCount"] = frame % 0x10
  values["CF_Lkas_Chksum"] = 0

  if values["CF_Lkas_LdwsOpt_USM"] == 4:
    values["CF_Lkas_LdwsOpt_USM"] = 3

  if lfa_available:
    values["CF_Lkas_LdwsActivemode"] = int(left_lane) + (int(right_lane) << 1)
    values["CF_Lkas_LdwsOpt_USM"] = 2

    # FcwOpt_USM 5 = Orange blinking car + lanes
    # FcwOpt_USM 4 = Orange car + lanes
    # FcwOpt_USM 3 = Green blinking car + lanes
    # FcwOpt_USM 2 = Green car + lanes
    # FcwOpt_USM 1 = White car + lanes
    # FcwOpt_USM 0 = No car + lanes
    values["CF_Lkas_FcwOpt_USM"] = 2 if enabled else 1

    # SysWarning 4 = keep hands on wheel
    # SysWarning 5 = keep hands on wheel (red)
    # SysWarning 6 = keep hands on wheel (red) + beep
    # Note: the warning is hidden while the blinkers are on
    values["CF_Lkas_SysWarning"] = 4 if sys_warning else 0

  elif car_fingerprint == CAR.HYUNDAI_GENESIS:
    # This field is actually LdwsActivemode
    # Genesis and Optima fault when forwarding while engaged
    values["CF_Lkas_LdwsActivemode"] = 2
  elif car_fingerprint == CAR.KIA_OPTIMA:
    values["CF_Lkas_LdwsActivemode"] = 0

  dat = packer.make_can_msg("LKAS11", 0, values)[2]

  if car_fingerprint in CHECKSUM["crc8"]:
    # CRC Checksum as seen on 2019 Hyundai Santa Fe
    dat = dat[:6] + dat[7:8]
    checksum = hyundai_checksum(dat)
  elif car_fingerprint in CHECKSUM["6B"]:
    # Checksum of first 6 Bytes, as seen on 2018 Kia Sorento
    checksum = sum(dat[:6]) % 256
  else:
    # Checksum of first 6 Bytes and last Byte as seen on 2018 Kia Stinger
    checksum = (sum(dat[:6]) + dat[7]) % 256

  values["CF_Lkas_Chksum"] = checksum

  return packer.make_can_msg("LKAS11", bus, values)


def create_clu11(packer, bus, clu11, button, speed, cnt):
  values = clu11

  if bus != 1:
    values["CF_Clu_CruiseSwState"] = button
    values["CF_Clu_Vanz"] = speed
  else:
    values["CF_Clu_Vanz"] = speed
  values["CF_Clu_AliveCnt1"] = cnt
  return packer.make_can_msg("CLU11", bus, values)

def create_lfa_mfa(packer, frame, enabled):
  values = {
    "ACTIVE": enabled,
    "HDA_USM": 2,
  }

  # ACTIVE 1 = Green steering wheel icon

  # LFA_USM 2 & 3 = LFA cancelled, fast loud beeping
  # LFA_USM 0 & 1 = No mesage

  # LFA_SysWarning 1 = "Switching to HDA", short beep
  # LFA_SysWarning 2 = "Switching to Smart Cruise control", short beep
  # LFA_SysWarning 3 =  LFA error

  # ACTIVE2: nothing
  # HDA_USM: nothing

  return packer.make_can_msg("LFAHDA_MFC", 0, values)

def create_scc11(packer, enabled, set_speed, lead_visible, lead_dist, lead_vrel, lead_yrel, gapsetting, standstill,
                 scc11, usestockscc, nosccradar, scc11cnt, sendaccmode):
  values = scc11

  if not usestockscc:
    if enabled:
      values["VSetDis"] = set_speed
    if standstill:
      values["SCCInfoDisplay"] = 0
    values["DriverAlertDisplay"] = 0
    values["TauGapSet"] = gapsetting
    values["ObjValid"] = lead_visible
    values["ACC_ObjStatus"] = lead_visible
    values["ACC_ObjRelSpd"] = lead_vrel
    values["ACC_ObjDist"] = lead_dist
    values["ACC_ObjLatPos"] = -lead_yrel

    if nosccradar:
      values["MainMode_ACC"] = sendaccmode
      values["AliveCounterACC"] = scc11cnt
  elif nosccradar:
    values["AliveCounterACC"] = scc11cnt

  return packer.make_can_msg("SCC11", 0, values)

def create_scc12(packer, apply_accel, enabled, standstill, gaspressed, brakepressed, aebcmdact, scc12,
                 usestockscc, nosccradar, cnt):
  values = scc12

  if not usestockscc and not aebcmdact:
    if enabled and not brakepressed:
      values["ACCMode"] = 2 if gaspressed and (apply_accel > -0.2) else 1
      if apply_accel < 0.0 and standstill:
        values["StopReq"] = 1
      values["aReqRaw"] = apply_accel
      values["aReqValue"] = apply_accel
    else:
      values["ACCMode"] = 0
      values["aReqRaw"] = 0
      values["aReqValue"] = 0

    if nosccradar:
      values["CR_VSM_Alive"] = cnt

    values["CR_VSM_ChkSum"] = 0
    dat = packer.make_can_msg("SCC12", 0, values)[2]
    values["CR_VSM_ChkSum"] = 16 - sum([sum(divmod(i, 16)) for i in dat]) % 16
  elif nosccradar:
    values["CR_VSM_Alive"] = cnt
    values["CR_VSM_ChkSum"] = 0
    dat = packer.make_can_msg("SCC12", 0, values)[2]
    values["CR_VSM_ChkSum"] = 16 - sum([sum(divmod(i, 16)) for i in dat]) % 16

  return packer.make_can_msg("SCC12", 0, values)

def create_scc13(packer, scc13):
  values = scc13
  return packer.make_can_msg("SCC13", 0, values)

def create_scc14(packer, enabled, usestockscc, aebcmdact, accel, scc14, objgap, gaspressed, standstill, e_vgo):
  values = scc14
  if not usestockscc and not aebcmdact:
    if enabled:
      values["ACCMode"] = 2 if gaspressed and (accel > -0.2) else 1
      values["ObjGap"] = objgap
      if standstill:
        values["JerkUpperLimit"] = 0.5
        values["JerkLowerLimit"] = 10.
        values["ComfortBandUpper"] = 0.
        values["ComfortBandLower"] = 0.
        if e_vgo > 0.27:
          values["ComfortBandUpper"] = 2.
          values["ComfortBandLower"] = 0.
      else:
        values["JerkUpperLimit"] = 50.
        values["JerkLowerLimit"] = 50.
        values["ComfortBandUpper"] = 50.
        values["ComfortBandLower"] = 50.

  return packer.make_can_msg("SCC14", 0, values)

def create_scc42a(packer):
  values = {
    "CF_FCA_Equip_Front_Radar": 1
  }
  return packer.make_can_msg("FRT_RADAR11", 0, values)

def create_fca11(packer, fca11, fca11cnt, fca11supcnt):
  values = fca11
  values["CR_FCA_Alive"] = fca11cnt
  values["Supplemental_Counter"] = fca11supcnt
  values["CR_FCA_ChkSum"] = 0
  dat = packer.make_can_msg("FCA11", 0, values)[2]
  values["CR_FCA_ChkSum"] = 16 - sum([sum(divmod(i, 16)) for i in dat]) % 16
  return packer.make_can_msg("FCA11", 0, values)

def create_fca12(packer):
  values = {
    "FCA_USM": 3,
    "FCA_DrvSetState": 2,
  }
  return packer.make_can_msg("FCA12", 0, values)

def create_mdps12(packer, frame, mdps12):
  values = mdps12
  if opParams().get('enableLKASbutton'):
    values["CF_Mdps_ToiActive"] = 0
    values["CF_Mdps_ToiUnavail"] = 1
    values["CF_Mdps_MsgCount2"] = frame % 0x100
    values["CF_Mdps_Chksum2"] = 0

    dat = packer.make_can_msg("MDPS12", 2, values)[2]
    checksum = sum(dat) % 256
    values["CF_Mdps_Chksum2"] = checksum

  return packer.make_can_msg("MDPS12", 2, values)

def create_scc7d0(cmd):
  return[2000, 0, cmd, 0]

