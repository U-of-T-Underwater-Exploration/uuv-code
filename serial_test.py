import serial
import time

ser = serial.Serial(
    port="/dev/ttyAMA1",
    baudrate=921600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

try:
    while True:
        data = ser.read(4096)
        print(f"{len(data)} bytes: {data.hex(' ')}")
        time.sleep(0.1)
finally:
    ser.close()