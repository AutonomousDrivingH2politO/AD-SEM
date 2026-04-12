import pytest
import can
from unittest.mock import MagicMock, patch

import ad_juno.test_files.source_can_ECU as can_bus


# -------------------------------------------------------------
# Config tests
# -------------------------------------------------------------

def test_can_ids_values():
    assert can_bus.CAN_IDS["speed"] == 0x602
    assert can_bus.CAN_IDS["emergency"] == 0x11
    assert can_bus.CAN_IDS["rpm"] == 0x600
    assert can_bus.CAN_IDS["throttle"] == 0x640
    assert can_bus.CAN_IDS["enable"] == 0x049


def test_can_filter_structure():
    assert len(can_bus.CAN_FILTER) == 1
    f = can_bus.CAN_FILTER[0]
    assert f["can_id"] == 0x602
    assert f["can_mask"] == 0x602
    assert f["extended"] is False


# -------------------------------------------------------------
# Bus creation
# -------------------------------------------------------------

def test_create_bus_uses_correct_args():
    with patch("can_bus.can.interface.Bus") as MockBus:
        bus = can_bus.create_bus()

        MockBus.assert_called_once_with(channel="can0", bustype="socketcan")
        assert bus is MockBus.return_value


# -------------------------------------------------------------
# Data serialisation
# -------------------------------------------------------------

def test_build_data_array_is_8_bytes():
    for value in [0, 1, 255, 256, 65535]:
        result = can_bus.build_data_array(value)
        assert isinstance(result, bytearray)
        assert len(result) == 8


def test_build_data_array_zero():
    result = can_bus.build_data_array(0)
    assert result == bytearray(8)


def test_build_data_array_max_uint16():
    result = can_bus.build_data_array(65535)
    assert result == bytearray([0xFF, 0xFF, 0, 0, 0, 0, 0, 0])


def test_build_data_array_big_endian():
    result = can_bus.build_data_array(0x0102)
    assert result[0] == 0x01
    assert result[1] == 0x02


def test_build_data_array_zero_padding():
    result = can_bus.build_data_array(1)
    assert result[2:] == bytearray(6)


def test_build_data_array_raises_for_negative():
    with pytest.raises(OverflowError):
        can_bus.build_data_array(-1)


def test_build_data_array_raises_for_too_large():
    with pytest.raises(OverflowError):
        can_bus.build_data_array(65536)


# -------------------------------------------------------------
# Message creation
# -------------------------------------------------------------

def test_build_message_uses_correct_fields():
    with patch("can_bus.can.Message") as MockMessage:
        msg = can_bus.build_message(0x049, 65535)

        MockMessage.assert_called_once()
        _, kwargs = MockMessage.call_args

        assert kwargs["arbitration_id"] == 0x049
        assert kwargs["is_extended_id"] is False
        assert kwargs["data"] == bytearray([0xFF, 0xFF, 0, 0, 0, 0, 0, 0])
        assert msg is MockMessage.return_value


# -------------------------------------------------------------
# Sending logic
# -------------------------------------------------------------

def test_send_can_calls_bus_send():
    fake_bus = MagicMock()

    with patch("can_bus.build_message") as mock_build:
        fake_msg = MagicMock()
        mock_build.return_value = fake_msg

        can_bus.send_can(fake_bus, 0x049, 65535)

        mock_build.assert_called_once_with(0x049, 65535)
        fake_bus.send.assert_called_once_with(fake_msg)


def test_send_can_propagates_can_error():
    fake_bus = MagicMock()
    fake_bus.send.side_effect = can.CanError("bus off")

    with patch("can_bus.build_message", return_value=MagicMock()):
        with pytest.raises(can.CanError, match="bus off"):
            can_bus.send_can(fake_bus, 0x049, 65535)


def test_send_can_propagates_unknown_error():
    fake_bus = MagicMock()
    fake_bus.send.side_effect = RuntimeError("unexpected failure")

    with patch("can_bus.build_message", return_value=MagicMock()):
        with pytest.raises(RuntimeError, match="unexpected failure"):
            can_bus.send_can(fake_bus, 0x049, 65535)


def test_send_can_prints_success_message():
    fake_bus = MagicMock()
    fake_msg = MagicMock()
    fake_msg.data = bytearray([0xFF, 0xFF, 0, 0, 0, 0, 0, 0])

    with patch("can_bus.build_message", return_value=fake_msg), \
         patch("builtins.print") as mock_print:
        can_bus.send_can(fake_bus, 0x049, 65535)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert any("Messaggio inviato con ID" in text for text in printed)


# -------------------------------------------------------------
# Script runner
# -------------------------------------------------------------

def test_run_enable_test_happy_path():
    fake_bus = MagicMock()
    fake_bus.state = "ACTIVE"
    fake_bus.recv = False

    with patch("can_bus.create_bus", return_value=fake_bus) as mock_create, \
         patch("can_bus.send_can") as mock_send, \
         patch("builtins.print") as mock_print:

        can_bus.run_enable_test()

    mock_create.assert_called_once()
    mock_send.assert_called_once_with(fake_bus, can_bus.CAN_IDS["enable"], 65535)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "ACTIVE" in printed
    assert "Messaggio 'enable' inviato." in printed