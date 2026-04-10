#!/usr/bin/env python3
import pytest
from unittest.mock import patch, call

import ad_juno.test_files.source_gpio_monitor as gpio_monitor


def test_constants_are_correct():
    assert gpio_monitor.INPUT_PIN == 18
    assert gpio_monitor.POLL_INTERVAL_S == 0.1


def test_setup_gpio_calls_correct_gpio_functions():
    with patch("gpio_monitor.GPIO.setwarnings") as mock_warnings, \
         patch("gpio_monitor.GPIO.setmode") as mock_setmode, \
         patch("gpio_monitor.GPIO.setup") as mock_setup:

        gpio_monitor.setup_gpio(18)

    mock_warnings.assert_called_once_with(False)
    mock_setmode.assert_called_once_with(gpio_monitor.GPIO.BCM)
    mock_setup.assert_called_once_with(
        18,
        gpio_monitor.GPIO.IN,
        pull_up_down=gpio_monitor.GPIO.PUD_DOWN,
    )


def test_cleanup_gpio_calls_gpio_cleanup():
    with patch("gpio_monitor.GPIO.cleanup") as mock_cleanup:
        gpio_monitor.cleanup_gpio()

    mock_cleanup.assert_called_once()


def test_read_pin_returns_gpio_input_value():
    with patch("gpio_monitor.GPIO.input", return_value=1) as mock_input:
        result = gpio_monitor.read_pin(18)

    mock_input.assert_called_once_with(18)
    assert result == 1


def test_format_state_message_high():
    result = gpio_monitor.format_state_message(True)
    assert result == "Initial state: HIGH (3.3V)"


def test_format_state_message_low():
    result = gpio_monitor.format_state_message(False)
    assert result == "Initial state: LOW (0V)"


def test_handle_state_change_high_prints_engaged_messages():
    with patch("builtins.print") as mock_print:
        gpio_monitor.handle_state_change(True)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "POWER DETECTED! (3.3V on pin)\n" in printed
    assert "System would be ENGAGED" in printed


def test_handle_state_change_low_prints_disengaged_messages():
    with patch("builtins.print") as mock_print:
        gpio_monitor.handle_state_change(False)

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "Power removed (0V on pin)\n" in printed
    assert "System would be DISENGAGED" in printed


def test_process_pin_state_returns_same_state_if_unchanged():
    with patch("gpio_monitor.handle_state_change") as mock_handle:
        result = gpio_monitor.process_pin_state(False, False)

    assert result is False
    mock_handle.assert_not_called()


def test_process_pin_state_updates_state_if_changed_to_high():
    with patch("gpio_monitor.handle_state_change") as mock_handle:
        result = gpio_monitor.process_pin_state(False, True)

    assert result is True
    mock_handle.assert_called_once_with(True)


def test_process_pin_state_updates_state_if_changed_to_low():
    with patch("gpio_monitor.handle_state_change") as mock_handle:
        result = gpio_monitor.process_pin_state(True, False)

    assert result is False
    mock_handle.assert_called_once_with(False)


def test_monitor_loop_reads_initial_state_and_processes_changes():
    with patch("gpio_monitor.read_pin", side_effect=[False, True]), \
         patch("gpio_monitor.process_pin_state", return_value=True) as mock_process, \
         patch("gpio_monitor.time.sleep", side_effect=KeyboardInterrupt), \
         patch("builtins.print") as mock_print:

        with pytest.raises(KeyboardInterrupt):
            gpio_monitor.monitor_loop(pin=18, poll_interval_s=0.1)

    mock_process.assert_called_once_with(False, True)
    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert "Initial state: LOW (0V)" in printed


def test_main_sets_up_and_cleans_up_gpio_on_keyboard_interrupt():
    with patch("gpio_monitor.setup_gpio") as mock_setup, \
         patch("gpio_monitor.monitor_loop", side_effect=KeyboardInterrupt), \
         patch("gpio_monitor.cleanup_gpio") as mock_cleanup, \
         patch("builtins.print") as mock_print:

        gpio_monitor.main()

    mock_setup.assert_called_once_with(gpio_monitor.INPUT_PIN)
    mock_cleanup.assert_called_once()

    printed = [args[0] for args, _ in mock_print.call_args_list]
    assert f"Monitoring GPIO pin {gpio_monitor.INPUT_PIN} for 3.3V power..." in printed
    assert "Press CTRL+C to exit" in printed
    assert "\nExiting program" in printed


def test_main_cleans_up_gpio_even_if_monitor_loop_fails():
    with patch("gpio_monitor.setup_gpio"), \
         patch("gpio_monitor.monitor_loop", side_effect=RuntimeError("GPIO read error")), \
         patch("gpio_monitor.cleanup_gpio") as mock_cleanup:

        with pytest.raises(RuntimeError, match="GPIO read error"):
            gpio_monitor.main()

    mock_cleanup.assert_called_once()