import minimalmodbus
import time
import csv
from datetime import datetime

# Create an instrument instance
instrument = minimalmodbus.Instrument('/dev/ttyUSB0', 11)  # Sensor address 0x0B = 11

# Serial configuration
instrument.serial.baudrate = 9600
instrument.serial.bytesize = 8
instrument.serial.parity = minimalmodbus.serial.PARITY_NONE
instrument.serial.stopbits = 1
instrument.serial.timeout = 1  # 1 second timeout

# RTU mode
instrument.mode = minimalmodbus.MODE_RTU

# CSV file path
csv_file = "chlorophyll_data.csv"

# Write header if file doesn't exist
try:
    with open(csv_file, 'x', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Timestamp", "Chlorophyll (ug/L)", "Temperature (°C)"])
except FileExistsError:
    pass  # File exists, no need to write header

print("Starting chlorophyll data logging every 20 seconds... (Ctrl+C to stop)")

try:
    while True:
        try:
            # Read 4 registers starting from address 0
            result = instrument.read_registers(0, 4, functioncode=3)

            # Parse data
            value_int = result[0]
            value_exp = result[1]
            temp_int = result[2]
            temp_exp = result[3]

            # Calculate actual values
            chlorophyll = value_int / (10 ** value_exp)
            temperature = temp_int / (10 ** temp_exp)

            # Timestamp
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Log to CSV
            with open(csv_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([timestamp, f"{chlorophyll:.4f}", f"{temperature:.2f}"])

            print(f"{timestamp} | Chlorophyll: {chlorophyll:.4f} ug/L | Temp: {temperature:.2f} °C")

        except Exception as e:
            print("Read failed:", e)

        time.sleep(20)

except KeyboardInterrupt:
    print("\nLogging stopped by user.")
