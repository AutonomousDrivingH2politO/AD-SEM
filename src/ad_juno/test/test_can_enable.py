import pytest
import can
import subprocess
from unittest.mock import MagicMock, patch

import ad_juno.test_files.source_can_enable as can_enable

# -------------------------------------------------------------
# Constants
# -------------------------------------------------------------

def test_can_ids_values():
    assert can_enable.CAN_IDS["speed"] == 0x602
    assert can_enable.CAN_IDS["emergency"] == 0x11
    assert can_enable.CAN_IDS["rpm"] == 0x600
    assert can_enable.CAN_IDS["throttle"] == 0x640
    assert can_enable.CAN_IDS["enable"] == 0x21


def test_can_filter_structure():
    assert len(can_enable.CAN_FILTER) == 1
    f = can_enable.CAN_FILTER[0]
    assert f["can_id"] == 0x602
    assert f["can_mask"] == 0x602
    assert f["extended"] is False


# -------------------------------------------------------------
# Bus creation
# -------------------------------------------------------------

def test_create_bus_uses_correct_args():
    with patch("can_enable.can.interface.Bus") as MockBus:
        bus = can_enable.create_bus()

        MockBus.assert_called_once_with(channel="can0", interface="socketcan")
        assert bus is MockBus.return_value


# -------------------------------------------------------------
# Payload / message building
# -------------------------------------------------------------

def test_build_data_array_zero():
    assert can_enable.build_data_array(0) == bytearray(8)


def test_build_data_array_is_8_bytes():
    for value in [0, 1, 255, 256, 65535]:
        assert len(can_enable.build_data_array(value)) == 8


def test_build_data_array_big_endian():
    payload = can_enable.build_data_array(0x0102)
    assert payload[0] == 0x01
    assert payload[1] == 0x02


def test_build_data_array_max_uint16():
    payload = can_enable.build_data_array(65535)
    assert payload == bytearray([0xFF, 0xFF, 0, 0, 0, 0, 0, 0])


def test_build_data_array_zero_padded():
    payload = can_enable.build_data_array(1)
    assert payload[2:] == bytearray(6)


def test_build_data_array_negative_raises():
    with pytest.raises(OverflowError):
        can_enable.build_data_array(-1)


def test_build_data_array_too_large_raises():
    with pytest.raises(OverflowError):
        can_enable.build_data_array(65536)


def test_build_message_uses_correct_fields():
    with patch("can_enable.can.Message") as MockMessage:
        msg = can_enable.build_message(0x21, 65535)

        MockMessage.assert_called_once()
        _, kwargs = MockMessage.call_args
        assert kwargs["arbitration_id"] == 0x21
        assert kwargs["is_extended_id"] is False
        assert kwargs["data"] == bytearray([0xFF, 0xFF, 0, 0, 0, 0, 0, 0])
        assert msg is MockMessage.return_value


# -------------------------------------------------------------
# send_can
# -------------------------------------------------------------

def test_send_can_calls_bus_send():
    fake_bus = MagicMock()
    fake_msg = MagicMock()

    with patch("can_enable.build_message", return_value=fake_msg) as mock_build:
        can_enable.send_can(fake_bus, 0x21, 65535)

    mock_build.assert_called_once_with(0x21, 65535)
    fake_bus.send.assert_called_once_with(fake_msg)


def test_send_can_prints_success_message():
    fake_bus = MagicMock()
    fake_msg = MagicMock()
    fake_msg.data = bytearray([0xFF, 0xFF, 0, 0, 0, 0, 0, 0])

    with patch("can_enable.build_message", return_value=fake_msg), \
         patch("builtins.print") as mock_print:
        can_enable.send_can(fake_bus, 0x21, 65535)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("Messaggio inviato con ID: 0x21" in text for text in printed)


def test_send_can_propagates_can_error():
    fake_bus = MagicMock()
    fake_bus.send.side_effect = can.CanError("bus off")

    with patch("can_enable.build_message", return_value=MagicMock()):
        with pytest.raises(can.CanError, match="bus off"):
            can_enable.send_can(fake_bus, 0x21, 65535)


def test_send_can_propagates_unknown_error():
    fake_bus = MagicMock()
    fake_bus.send.side_effect = RuntimeError("unexpected failure")

    with patch("can_enable.build_message", return_value=MagicMock()):
        with pytest.raises(RuntimeError, match="unexpected failure"):
            can_enable.send_can(fake_bus, 0x21, 65535)


# -------------------------------------------------------------
# close_can
# -------------------------------------------------------------

def test_close_can_returns_if_interface_does_not_exist():
    fake_result = MagicMock()
    fake_result.returncode = 1

    with patch("can_enable.subprocess.run", return_value=fake_result) as mock_run, \
         patch("builtins.print") as mock_print:
        can_enable.close_can("can9")

    mock_run.assert_called_once()
    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("does not exist" in text for text in printed)


def test_close_can_brings_interface_down_when_present():
    first_result = MagicMock()
    first_result.returncode = 0

    with patch("can_enable.subprocess.run", side_effect=[first_result, None]) as mock_run, \
         patch("builtins.print") as mock_print:
        can_enable.close_can("can0")

    assert mock_run.call_count == 2

    first_call = mock_run.call_args_list[0]
    second_call = mock_run.call_args_list[1]

    assert first_call.args[0] == ["ip", "link", "show", "can0"]
    assert second_call.args[0] == ["sudo", "ip", "link", "set", "can0", "down"]

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("Closing CAN interface: can0..." in text for text in printed)
    assert any("has been closed properly" in text for text in printed)


def test_close_can_handles_called_process_error():
    first_result = MagicMock()
    first_result.returncode = 0

    with patch(
        "can_enable.subprocess.run",
        side_effect=[first_result, subprocess.CalledProcessError(1, "ip")]
    ), patch("builtins.print") as mock_print:
        can_enable.close_can("can0")

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("Error while closing CAN interface can0" in text for text in printed)


def test_close_can_handles_unknown_exception():
    with patch("can_enable.subprocess.run", side_effect=Exception("boom")), \
         patch("builtins.print") as mock_print:
        can_enable.close_can("can0")

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("Unexpected error: boom" in text for text in printed)


# -------------------------------------------------------------
# Script runner
# -------------------------------------------------------------

def test_run_enable_test_creates_bus_and_sends_enable():
    fake_bus = MagicMock()
    fake_bus.state = "ACTIVE"

    with patch("can_enable.create_bus", return_value=fake_bus) as mock_create, \
         patch("can_enable.send_can") as mock_send, \
         patch("builtins.print") as mock_print:

        can_enable.run_enable_test()

    mock_create.assert_called_once()
    mock_send.assert_called_once_with(fake_bus, can_enable.CAN_IDS["enable"], 65535)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "ACTIVE" in printed
    assert "Messaggio 'enable' inviato." in printed