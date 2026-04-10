import can

CAN_IDS = {
    "speed": 0x602,
    "emergency": 0x11,
    "rpm": 0x600,
    "throttle": 0x640,
    "enable": 0x049,
}

CAN_FILTER = [{"can_id": 0x602, "can_mask": 0x602, "extended": False}]


def create_bus(channel: str = "can0", bustype: str = "socketcan"):
    return can.interface.Bus(channel=channel, bustype=bustype)


def build_data_array(data: int) -> bytearray:
    data_array = list(data.to_bytes(2, byteorder="big"))
    for _ in range(8 - len(data_array)):
        data_array.append(0)
    return bytearray(data_array)


def build_message(can_id: int, data: int):
    data_array = build_data_array(data)
    return can.Message(
        arbitration_id=can_id,
        data=data_array,
        is_extended_id=False,
    )


def send_can(bus, can_id: int, data: int) -> None:
    try:
        msg = build_message(can_id, data)
        bus.send(msg)
        print(f"Message sent with ID: {hex(can_id)}, Data: {msg.data}")
    except can.CanError as e:
        print(f"Error sending CAN message: {e}")
        raise
    except Exception as e:
        print(f"Unknown error: {e}")
        raise


def run_enable_test():
    bus = create_bus()
    print(bus.state)

    send_can(bus, CAN_IDS["enable"], 65535)

    if bus.recv == True:
        print(bus.recv)

    print("Message 'enable' sent.")


if __name__ == "__main__":
    run_enable_test()


