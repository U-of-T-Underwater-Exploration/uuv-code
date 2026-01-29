from uuv_baro_ext import ms5837
import time

sensor = ms5837.MS5837_30BA(6) # I2C bus is 6

# We must initialize the sensor before reading it
if not sensor.init():
        print("Sensor could not be initialized")
        exit(1)

# We have to read values from sensor to update pressure and temperature
if not sensor.read():
    print("Sensor read failed!")
    exit(1)

print(f"Pressure: {sensor.pressure(ms5837.UNITS_atm)} atm")


print(f"Temperature: {sensor.temperature(ms5837.UNITS_Centigrade)} C")

freshwaterDepth = sensor.depth()
print(f"Depth: {freshwaterDepth} m (freshwater)")

print(f"MSL Relative Altitude: {sensor.altitude()} m")

time.sleep(5)

# Output readings
while True:
        if sensor.read():
                print(f"P: {sensor.pressure()}\nC: {sensor.temperature()}")
        else:
                print("Sensor read failed!")
                exit(1)