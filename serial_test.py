import struct
import time

import serial

PORT = "/dev/ttyAMA1"
TARGET_BAUD = 115200

# ID_UART BAUDRATE range in the Kogger SBP spec is 9600 ... 921600
BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
SCAN_SECONDS = 1.5

ID_UART = 0x18
ID_DVL_VEL = 0x79
KEY_CONFIRM = 0xC96B5D4A
RESP_CODES = {
    0: "RESP_NONE",
    1: "RESP_OK",
    2: "RESP_ERR_CHECKSUMM",
    3: "RESP_ERR_PAYLOAD",
    4: "RESP_ERR_ID",
    5: "RESP_ERR_VERSION",
    6: "RESP_ERR_TYPE",
    7: "RESP_ERR_KEY",
    8: "RESP_ERR_RUNTIME",
}


def fletcher16(data):
    check1 = 0
    check2 = 0
    for byte in data:
        check1 = (check1 + byte) & 0xFF
        check2 = (check2 + check1) & 0xFF
    return check1, check2


def build_frame(mode, frame_id, payload, route=0x00):
    body = bytes([route, mode, frame_id, len(payload)]) + payload
    return b"\xBB\x55" + body + bytes(fletcher16(body))


def parse_frames(buffer):
    """Return (frames, leftover) where each frame is (mode, id, payload) with a valid checksum."""
    frames = []
    i = 0
    while len(buffer) - i >= 8:
        if buffer[i] != 0xBB or buffer[i + 1] != 0x55:
            i += 1
            continue
        length = buffer[i + 5]
        if length > 128:
            i += 1
            continue
        frame_size = 6 + length + 2
        if len(buffer) - i < frame_size:
            break
        check1, check2 = fletcher16(buffer[i + 2:i + 6 + length])
        if check1 != buffer[i + 6 + length] or check2 != buffer[i + 7 + length]:
            i += 1
            continue
        mode = buffer[i + 3]
        frame_id = buffer[i + 4]
        payload = bytes(buffer[i + 6:i + 6 + length])
        frames.append((mode, frame_id, payload))
        i += frame_size
    return frames, buffer[i:]


def open_port(baud):
    return serial.Serial(
        port=PORT,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.1,
    )


def read_frames(ser, seconds):
    buffer = bytearray()
    frames = []
    total_bytes = 0
    deadline = time.time() + seconds
    while time.time() < deadline:
        data = ser.read(4096)
        total_bytes += len(data)
        buffer += data
        new_frames, buffer = parse_frames(buffer)
        frames += new_frames
    return frames, total_bytes


def scan():
    best_baud = None
    best_count = 0
    for baud in BAUD_RATES:
        with open_port(baud) as ser:
            ser.reset_input_buffer()
            frames, total_bytes = read_frames(ser, SCAN_SECONDS)
        dvl_count = sum(1 for _, frame_id, _ in frames if frame_id == ID_DVL_VEL)
        ids = sorted({frame_id for _, frame_id, _ in frames})
        print(f"{baud:>7} baud: {total_bytes:6} bytes, {len(frames):4} valid frames, "
              f"{dvl_count:4} ID_DVL_VEL, ids={[hex(x) for x in ids]}")
        if len(frames) > best_count:
            best_baud = baud
            best_count = len(frames)
    return best_baud


def set_baud(current_baud, new_baud):
    # MODE: TYPE=2 (SETTING), VERSION=0, RESPONSE=1 so the device replies with RESP
    mode = 0x02 | (0 << 3) | 0x80
    # Format [U4 KEY_CONFIRM, U1 UART_ID, U4 BAUDRATE]
    payload = struct.pack("<IBI", KEY_CONFIRM, 1, new_baud)
    frame = build_frame(mode, ID_UART, payload)
    print(f"Sending ID_UART at {current_baud} baud: {frame.hex(' ')}")

    with open_port(current_baud) as ser:
        ser.reset_input_buffer()
        ser.write(frame)
        ser.flush()
        frames, _ = read_frames(ser, 0.5)

    for mode, frame_id, payload in frames:
        if frame_id == ID_UART and mode & 0x80 and len(payload) >= 1:
            print(f"Device response: {RESP_CODES.get(payload[0], payload[0])}")
            break
    else:
        print("No RESP received (the device may have already switched baud rate)")


def main():
    print(f"Scanning {PORT} ...")
    current_baud = scan()
    if current_baud is None:
        print("No valid SBP frames at any baud rate. Is the DVL powered and streaming?")
        return
    print(f"Detected DVL at {current_baud} baud")

    if current_baud != TARGET_BAUD:
        set_baud(current_baud, TARGET_BAUD)
        time.sleep(0.5)

    print(f"Verifying at {TARGET_BAUD} baud ...")
    with open_port(TARGET_BAUD) as ser:
        ser.reset_input_buffer()
        frames, total_bytes = read_frames(ser, SCAN_SECONDS)
    dvl_count = sum(1 for _, frame_id, _ in frames if frame_id == ID_DVL_VEL)
    print(f"{TARGET_BAUD} baud: {total_bytes} bytes, {len(frames)} valid frames, "
          f"{dvl_count} ID_DVL_VEL")


if __name__ == "__main__":
    main()
