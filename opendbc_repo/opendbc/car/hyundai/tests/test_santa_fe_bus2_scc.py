from unittest.mock import patch

import pytest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import get_car
from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.discovery import CLASSIC_NORMAL_BUS_LAYOUT, SANTA_FE_BUS2_LAYOUT
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR, DBC, HyundaiSafetyFlags, santa_fe_bus2_scc
from opendbc.car.vin import VIN_UNKNOWN


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


def test_uses_dedicated_safety_and_leaves_stock_ecu_running():
  cp = params()
  assert santa_fe_bus2_scc(cp)
  assert cp.openpilotLongitudinalControl
  assert cp.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.hyundaiSantaFeBus2
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
  assert stock.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.hyundai
  assert CarInterface.startup_required(stock)

  retrofit = params(alpha=False)
  assert santa_fe_bus2_scc(retrofit)
  assert not retrofit.openpilotLongitudinalControl
  assert retrofit.safetyConfigs[0].safetyModel == structs.CarParams.SafetyModel.hyundai
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


@pytest.mark.parametrize(('wiring', 'selected', 'normal_query'), [
  ('retrofit', True, True), ('retrofit', False, False), ('factory', True, True),
  ('missing_fca', True, False), ('mixed_scc', True, False), ('malformed', True, False),
])
def test_manual_retrofit_uses_normal_diagnostics_with_live_bus_evidence(wiring, selected, normal_query):
  layout = dict(CLASSIC_NORMAL_BUS_LAYOUT if wiring == 'factory' else SANTA_FE_BUS2_LAYOUT)
  if wiring == 'missing_fca':
    del layout[(2, 0x38D)]
  elif wiring == 'mixed_scc':
    layout[(0, 0x421)] = 8
  elif wiring == 'malformed':
    layout[(2, 0x421)] = 7
  traffic = [CanData(address, bytes(size), bus) for (bus, address), size in layout.items()]
  switches = []
  with patch('opendbc.car.car_helpers.get_vin', return_value=(-1, -1, VIN_UNKNOWN)), \
       patch('opendbc.car.car_helpers.get_fw_versions', return_value=[]) as normal, \
       patch('opendbc.car.car_helpers.get_present_ecus', return_value=set()) as present, \
       patch('opendbc.car.car_helpers.get_fw_versions_ordered', return_value=[]) as generic:
    ci = get_car(lambda **_: [traffic], lambda _: None, switches.append, False, False,
                 forced_candidate=CAR.HYUNDAI_SANTA_FE if selected else None)

  if normal_query:
    normal.assert_called_once()
    assert normal.call_args.kwargs == {'query_brand': 'hyundai', 'normal_bus_only': True}
    present.assert_not_called()
    generic.assert_not_called()
    assert switches == [False, False]
  else:
    normal.assert_not_called()
    generic.assert_called_once()
    assert switches == [True, False]
  if selected:
    assert ci.CP.carFingerprint == CAR.HYUNDAI_SANTA_FE
    assert ci.CP.fingerprintSource == structs.CarParams.FingerprintSource.fixed
  assert not ci.CP.openpilotLongitudinalControl


def test_only_proposes_acceleration_without_sending_display_or_fca():
  ci = CarInterface(params())
  ci.update([(CAPTURE_NS, stock_frames())])
  _, frames = ci.apply(control_request().as_reader(), CAPTURE_NS)
  parser = CANParser(DBC_NAME, [("SCC12", 50)], 0)
  assert parser.update([(CAPTURE_NS, frames)]) == {0x421}
  assert parser.vl["SCC12"]["ACCMode"] == 1
  assert parser.vl["SCC12"]["aReqValue"] == pytest.approx(-0.5)
  assert not any(address in (0x420, 0x389, 0x50A, 0x38D, 0x483, 0x4A2, 0x7D0) for address, _, _ in frames)
  scc12 = next(data for address, data, _ in frames if address == 0x421)
  assert sum(sum(divmod(byte, 16)) for byte in scc12) % 16 == 0


def test_missing_and_stale_stock_frames_inhibit_long_commands():
  ci = CarInterface(params())
  assert not ci.CS.santa_fe_scc_current
  ci.update([(CAPTURE_NS, stock_frames())])
  assert ci.CS.santa_fe_scc_current
  expired = CAPTURE_NS + 250_000_000
  ci.update([(expired, [])])
  assert not ci.CS.santa_fe_scc_current
  assert ci.CS.out.accFaulted
  actuators, frames = ci.apply(control_request().as_reader(), expired)
  assert actuators.accel == 0
  assert not any(address == 0x421 for address, _, _ in frames)


@pytest.mark.parametrize("warning", [False, True])
def test_releases_long_commands_during_stock_aeb(warning):
  ci = CarInterface(params())
  ci.update([(CAPTURE_NS, stock_frames(braking=True, warning=warning))])
  assert ci.CS.out.stockAeb
  actuators, frames = ci.apply(control_request().as_reader(), CAPTURE_NS)
  assert actuators.accel == 0
  assert not any(address in (0x421, 0x38D) for address, _, _ in frames)


def test_generic_scc_output_without_stock_display():
  cp = params(bus=0)
  hud = structs.CarControl.HUDControl(leadVisible=True, leadDistanceBars=2)
  frames = hyundaican.create_acc_commands(CANPacker(DBC_NAME), True, -0.5, 3, 0, hud, 50, False, False, True, cp)
  assert decode_scc(frames)["SCC12"]["aReqValue"] == pytest.approx(-0.5)
