import minimalmodbus
import time
import csv
from datetime import datetime

# Setup Modbus instrument
instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 1)
instrument.serial.baudrate = 4800
instrument.serial.bytesize = 8
instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 1
instrument.mode = minimalmodbus.MODE_RTU

# CSV file path
csv_file = "blue_green_algae_data.csv"

# Write CSV header if the file is new
try:
    with open(csv_file, 'x', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Timestamp", "BlueGreenAlgae (cells/mL)", "Temperature (°C)"])
except FileExistsError:
    pass  # File already exists; no need to write header

print("Starting data logging every 20 seconds...\nPress Ctrl+C to stop.")

try:
    while True:
        try:
            # Read Modbus registers
            result = instrument.read_registers(0, 4, functioncode=3)

            # Parse data
            value_int = result[0]
            value_exp = result[1]
            temp_int = result[2]

            # Calculate actual values
            algae = value_int / (10 ** value_exp)
            temp = temp_int / 10

            # Get timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Append data to CSV
            with open(csv_file, 'a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([timestamp, f"{algae:.4f}", f"{temp:.2f}"])

            print(f"{timestamp} | Algae: {algae:.4f} cells/mL | Temp: {temp:.2f} °C")

        except Exception as e:
            print("Read failed:", e)

        time.sleep(20)

except KeyboardInterrupt:
    print("\nLogging stopped by user.")
