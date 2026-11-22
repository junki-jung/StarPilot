import pytest

from opendbc.can import CANPacker
from opendbc.car.hyundai.hyundaican import create_scc12_request, hyundai_checksum
from opendbc.car.hyundai.values import Buttons, HyundaiSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.test_hyundai import checksum


class SantaFeBus2:
  def __init__(self):
    self.safety = libsafety_py.libsafety
    self.packer = CANPacker("hyundai_can_generated")
    flags = HyundaiSafetyFlags.CAMERA_SCC | HyundaiSafetyFlags.LONG
    assert self.safety.set_safety_hooks(CarParams.SafetyModel.hyundaiSantaFeBus2, flags) == 0
    self.safety.init_tests()
    self.safety.set_timer(100_000)
    self.button_counter = 0
    for name, values in (
      ("EMS16", {"AliveCounter": 1}), ("WHL_SPD11", {"WHL_SPD_AliveCounter_LSB": 1}),
      ("TCS13", {"AliveCounterTCS": 1}), ("MDPS12", {}),
    ):
      msg = self.packer.make_can_msg(name, 0, values)
      addr, data, bus = checksum(msg) if name != "MDPS12" else msg
      assert self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def button(self, button):
    self.button_counter += 1
    msg = self.packer.make_can_msg("CLU11", 0, {"CF_Clu_CruiseSwState": button, "CF_Clu_AliveCnt1": self.button_counter})
    addr, data, bus = msg
    assert self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))

  def scc(self, counter, **signals):
    values = {"ACCMode": 1, "CR_VSM_Alive": counter, "aReqRaw": -0.6, "aReqValue": -0.6, "CF_VSM_ConfMode": 2} | signals
    return checksum(self.packer.make_can_msg("SCC12", 2, values))

  def fca(self, counter, **signals):
    addr, data, bus = self.packer.make_can_msg("FCA11", 2, {"CR_FCA_Alive": counter} | signals)
    return addr, data[:7] + bytes([hyundai_checksum(data[:7])]), bus

  def forward(self, frame):
    addr, data, bus = frame
    packet = libsafety_py.make_CANPacket(addr, bus, data)
    assert self.safety.safety_fwd_packet_hook(packet) == 0
    self.safety.safety_prepare_fwd(packet)
    assert self.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, bus, data))
    return bytes(libsafety_py.ffi.buffer(packet.data, len(data)))

  def stage(self, accel=-1, *, stopping=False):
    addr, data, bus = create_scc12_request(self.packer, accel, stopping, 3)
    packet = libsafety_py.make_CANPacket(addr, bus, data)
    assert self.safety.safety_tx_hook(packet)
    assert self.safety.safety_stage_tx(packet)
    return data

  def engage(self):
    self.button(Buttons.SET_DECEL)
    self.button(Buttons.NONE)
    assert self.safety.get_controls_allowed()


@pytest.fixture
def bridge():
  bridge = SantaFeBus2()
  for counter in (13, 14):
    for frame in (bridge.fca(counter), bridge.scc(counter)):
      assert bridge.forward(frame) == frame[1]
  bridge.engage()
  return bridge


def test_keeps_stock_clock_counter_and_non_acceleration_fields(bridge):
  request = bridge.stage(stopping=True)
  bridge.forward(bridge.fca(0))
  original = bridge.scc(0, CF_VSM_Stat=1)[1]
  forwarded = bridge.forward((0x421, original, 2))
  assert forwarded[0] == original[0]
  assert forwarded[1] == original[1] | 0x80
  assert forwarded[2] == original[2]
  assert forwarded[3:6] == request[3:6]
  assert forwarded[6] == original[6]
  assert forwarded[7] & 0xF == 0
  assert sum(sum(divmod(v, 16)) for v in forwarded) % 16 == 0
  for addr in (0x420, 0x389, 0x50A, 0x483):
    assert bridge.forward((addr, b"\x12" * 8, 2)) == b"\x12" * 8
    assert not bridge.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, 0, b"\0" * 8))


@pytest.mark.parametrize("signal", ["AEB_CmdAct", "AEB_StopReq", "CF_VSM_DecCmdAct", "TakeOverReq", "StopReq"])
def test_stock_scc12_intervenes_even_with_pending_host_request(bridge, signal):
  bridge.stage()
  bridge.forward(bridge.fca(0))
  emergency = bridge.scc(0, **{signal: 1})
  assert bridge.forward(emergency) == emergency[1]
  assert not bridge.safety.get_controls_allowed()
  normal = bridge.scc(1)
  bridge.forward(bridge.fca(1))
  assert bridge.forward(normal) == normal[1]


@pytest.mark.parametrize("signal", ["FCA_CmdAct", "FCA_StopReq", "CF_VSM_Prefill", "CF_VSM_Warn", "FCA_Failinfo"])
def test_stock_fca11_disarms_pending_host_request(bridge, signal):
  bridge.stage()
  emergency = bridge.fca(0, **{signal: 1})
  assert bridge.forward(emergency) == emergency[1]
  normal = bridge.scc(0)
  assert bridge.forward(normal) == normal[1]
  assert not bridge.safety.get_controls_allowed()


@pytest.mark.parametrize("failure", ["scc_counter", "fca_repeat", "scc_checksum", "fca_checksum", "stale_host", "stale_fca", "missing_fca", "cancel"])
def test_falls_back_when_command_or_source_is_invalid(bridge, failure):
  bridge.stage()
  addr, data, bus = bridge.scc(2 if failure == "scc_counter" else 0)
  if failure == "scc_checksum":
    data = data[:-1] + bytes([data[-1] ^ 0x10])
  if failure in ("fca_repeat", "fca_checksum"):
    fca_id, fca_data, fca_bus = bridge.fca(14 if failure == "fca_repeat" else 0)
    if failure == "fca_checksum":
      fca_data = fca_data[:-1] + bytes([fca_data[-1] ^ 1])
    assert bridge.forward((fca_id, fca_data, fca_bus)) == fca_data
  if failure == "stale_host":
    bridge.safety.set_timer(140_001)
  if failure not in ("fca_repeat", "fca_checksum", "missing_fca"):
    bridge.forward(bridge.fca(0))
  if failure == "scc_counter":
    bridge.forward(bridge.fca(1))
    bridge.forward(bridge.fca(2))
  if failure == "stale_fca":
    bridge.safety.set_timer(115_001)
  if failure == "cancel":
    bridge.button(Buttons.CANCEL)
  assert bridge.forward((addr, data, bus)) == data
  assert not bridge.safety.get_controls_allowed()


def test_does_not_reuse_a_proposal_or_accept_host_emergency(bridge):
  bridge.stage()
  stock = bridge.scc(0)
  bridge.forward(bridge.fca(0))
  assert bridge.forward(stock) != stock[1]
  next_stock = bridge.scc(1)
  bridge.forward(bridge.fca(1))
  assert bridge.forward(next_stock) == next_stock[1]
  assert not bridge.safety.get_controls_allowed()
  bridge.engage()
  for frame in (bridge.scc(2, AEB_CmdAct=1), bridge.scc(2, aReqRaw=4, aReqValue=4)):
    addr, data, _ = frame
    assert not bridge.safety.safety_tx_hook(libsafety_py.make_CANPacket(addr, 0, data))


def test_relay_blocks_forwarding_if_scc12_appears_on_vehicle_bus(bridge):
  bridge.stage()
  addr, data, _ = bridge.scc(0)
  assert bridge.safety.safety_rx_hook(libsafety_py.make_CANPacket(addr, 0, data))
  assert bridge.safety.get_relay_malfunction()
  assert bridge.safety.safety_fwd_packet_hook(libsafety_py.make_CANPacket(addr, 2, data)) == -1
