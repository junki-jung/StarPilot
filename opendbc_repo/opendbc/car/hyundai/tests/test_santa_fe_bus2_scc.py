from unittest.mock import patch

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.carstate import santa_fe_stock_scc_display
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC, HyundaiSafetyFlags, santa_fe_bus2_scc


BUS2_MESSAGES = {0x420: 8, 0x421: 8, 0x389: 8, 0x38D: 8}
DBC_NAME = DBC[CAR.HYUNDAI_SANTA_FE][Bus.pt]
CAPTURE_NS = 1_000_000_000


def params(*, bus=2, missing=None, alpha=True):
  fingerprint = gen_empty_fingerprint()
  fingerprint[bus].update(BUS2_MESSAGES)
  if missing is not None:
    fingerprint[bus].pop(missing)
  return CarInterface.get_params(CAR.HYUNDAI_SANTA_FE, fingerprint, [], alpha, False, False)


def stock_frames(*, braking=False, warning=False):
  packer = CANPacker(DBC_NAME)
  fca = {"CF_VSM_Warn": 2 if warning else 0, "CF_VSM_DecCmdAct": int(braking), "FCA_CmdAct": int(braking)}
  return [
    packer.make_can_msg("SCC11", 2, {"ObjValid": 1, "ACC_ObjStatus": 1, "ACC_ObjDist": 35,
                                     "ACC_ObjRelSpd": -1, "ACC_ObjLatPos": 0.5}),
    packer.make_can_msg("SCC12", 2, {"ACCMode": 0}),
    packer.make_can_msg("SCC14", 2, {"ObjGap": 5, "ObjDistStat": 2}),
    packer.make_can_msg("FCA11", 2, fca),
  ]


def control_request():
  cc = structs.CarControl(enabled=True, longActive=True)
  cc.actuators.longControlState = structs.CarControl.Actuators.LongControlState.pid
  cc.actuators.accel = -0.5
  return cc


def decode_scc(frames):
  parser = CANParser(DBC_NAME, [("SCC11", 50), ("SCC12", 50), ("SCC14", 50)], 0)
  assert parser.update([(CAPTURE_NS, frames)]) == {0x420, 0x421, 0x389}
  return parser.vl


def test_uses_existing_camera_bus_safety_and_leaves_stock_ecu_running():
  cp = params()
  assert santa_fe_bus2_scc(cp)
  assert cp.openpilotLongitudinalControl
  assert cp.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.CAMERA_SCC
  assert cp.safetyConfigs[0].safetyParam & HyundaiSafetyFlags.LONG
  assert not CarInterface.startup_required(cp)
  with patch("opendbc.car.hyundai.interface.disable_ecu") as disable:
    CarInterface.init(cp, None, None)
    disable.assert_not_called()


def test_factory_wiring_and_alpha_choice_are_preserved():
  stock = params(bus=0)
  assert not santa_fe_bus2_scc(stock)
  assert stock.openpilotLongitudinalControl
  assert CarInterface.startup_required(stock)

  retrofit = params(alpha=False)
  assert santa_fe_bus2_scc(retrofit)
  assert not retrofit.openpilotLongitudinalControl
  assert retrofit.pcmCruise


@pytest.mark.parametrize("missing", BUS2_MESSAGES)
def test_incomplete_bus2_configuration_does_not_admit_long(missing):
  assert not params(missing=missing).openpilotLongitudinalControl


def test_ambiguous_scc_sources_do_not_admit_long():
  fingerprint = gen_empty_fingerprint()
  fingerprint[2].update(BUS2_MESSAGES)
  fingerprint[0][0x421] = 8
  cp = CarInterface.get_params(CAR.HYUNDAI_SANTA_FE, fingerprint, [], True, False, False)
  assert not cp.openpilotLongitudinalControl
  assert not santa_fe_bus2_scc(cp)


def test_reuses_display_without_reusing_stock_acceleration():
  ci = CarInterface(params())
  ci.update([(CAPTURE_NS, stock_frames())])
  _, frames = ci.apply(control_request().as_reader(), CAPTURE_NS)
  scc = decode_scc(frames)
  assert scc["SCC11"]["ObjValid"] == 1
  assert scc["SCC11"]["ACC_ObjDist"] == 35
  assert scc["SCC11"]["ACC_ObjRelSpd"] == pytest.approx(-1, abs=0.1)
  assert scc["SCC14"]["ObjGap"] == 5
  assert scc["SCC14"]["ObjDistStat"] == 2
  assert scc["SCC12"]["ACCMode"] == 1
  assert scc["SCC12"]["aReqValue"] == pytest.approx(-0.5)
  assert not any(address in (0x38D, 0x483, 0x7D0) for address, _, _ in frames)
  scc12 = next(data for address, data, _ in frames if address == 0x421)
  assert sum(sum(divmod(byte, 16)) for byte in scc12) % 16 == 0


def test_missing_and_stale_stock_frames_inhibit_long_commands():
  ci = CarInterface(params())
  assert santa_fe_stock_scc_display(ci.can_parsers[Bus.cam]) is None
  ci.update([(CAPTURE_NS, stock_frames())])
  assert ci.CS.stock_scc_display is not None
  expired = CAPTURE_NS + 250_000_000
  ci.update([(expired, [])])
  assert ci.CS.stock_scc_display is None
  assert ci.CS.out.accFaulted
  actuators, frames = ci.apply(control_request().as_reader(), expired)
  scc = decode_scc(frames)
  assert actuators.accel == 0
  assert scc["SCC12"]["ACCMode"] == 0
  assert scc["SCC12"]["aReqValue"] == pytest.approx(0)


@pytest.mark.parametrize("warning", [False, True])
def test_releases_long_commands_during_stock_aeb(warning):
  ci = CarInterface(params())
  ci.update([(CAPTURE_NS, stock_frames(braking=True, warning=warning))])
  assert ci.CS.out.stockAeb
  actuators, frames = ci.apply(control_request().as_reader(), CAPTURE_NS)
  scc = decode_scc(frames)
  assert actuators.accel == 0
  assert scc["SCC12"]["ACCMode"] == 0
  assert not any(address == 0x38D for address, _, _ in frames)


def test_generic_scc_output_without_stock_display():
  cp = params(bus=0)
  hud = structs.CarControl.HUDControl(leadVisible=True, leadDistanceBars=2)
  frames = hyundaican.create_acc_commands(CANPacker(DBC_NAME), True, -0.5, 3, 0, hud, 50, False, False, True, cp)
  assert decode_scc(frames)["SCC12"]["aReqValue"] == pytest.approx(-0.5)
