import can
import subprocess

CAN_IDS = {
    "speed": 0x602,
    "emergency": 0x11,
    "rpm": 0x600,
    "throttle": 0x640,
    "enable": 0x21,
}

CAN_FILTER = [{"can_id": 0x602, "can_mask": 0x602, "extended": False}]


def create_bus(channel: str = "can0", interface: str = "socketcan"):
    return can.interface.Bus(channel=channel, interface=interface)


def build_data_array(data: int) -> bytearray:
    data_array = list(data.to_bytes(2, byteorder="big"))
    for _ in range(8 - len(data_array)):
        data_array.append(0)
    return bytearray(data_array)


def build_message(can_id: int, data: int):
    return can.Message(
        arbitration_id=can_id,
        data=build_data_array(data),
        is_extended_id=False,
    )


def send_can(bus, can_id: int, data: int) -> None:
    try:
        msg = build_message(can_id, data)
        bus.send(msg)
        print(f"Messaggio inviato con ID: {hex(can_id)}, Dati: {msg.data}")
    except can.CanError as e:
        print(f"Errore nell'invio del messaggio CAN: {e}")
        raise
    except Exception as e:
        print(f"Errore sconosciuto: {e}")
        raise


def close_can(interface: str = "can1") -> None:
    try:
        result = subprocess.run(
            ["ip", "link", "show", interface],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        if result.returncode != 0:
            print(f"Error: The interface '{interface}' does not exist.")
            return

        print(f"Closing CAN interface: {interface}...")
        subprocess.run(
            ["sudo", "ip", "link", "set", interface, "down"],
            check=True,
        )
        print(f"CAN interface {interface} has been closed properly.")
    except subprocess.CalledProcessError as e:
        print(f"Error while closing CAN interface {interface}: {e}")
    except Exception as e:
        print(f"Unexpected error: {e}")


def run_enable_test():
    bus = create_bus()
    print(bus.state)
    send_can(bus, CAN_IDS["enable"], 65535)
    print("Messaggio 'enable' inviato.")


if __name__ == "__main__":
    run_enable_test()