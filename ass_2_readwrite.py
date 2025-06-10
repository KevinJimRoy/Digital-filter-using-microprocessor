import numpy as np
import serial
import time
import csv

port = 'COM5'       # Update with your port
baud_rate = 115200
fs = 1000           # Sampling rate (Hz)
num_samples = 1000  # Total samples
a1, a2, a3 = 127, 80, 8
output_csv = "filtered_vs_unfiltered.csv"

t = np.arange(num_samples) / fs
signal = a1 + a2 * np.sin(2 * np.pi * 10 * t) + a3 * np.sin(2 * np.pi * 100 * t)

signal_clipped = np.clip(signal, 0, 255)
signal_uint8 = signal_clipped.astype(np.uint8)

ser = serial.Serial(port, baud_rate, timeout=1)
print('Done1')
time.sleep(2)  # Let the MCU boot and initialize UART

filtered_signal = []

for sample in signal_uint8:
    ser.write(bytes([sample]))        # Send unfiltered byte
    print('Done2')
    received = ser.read()             # Read filtered byte
    print('Done3')
    if received:
        filtered_signal.append(ord(received))
    else:
        filtered_signal.append(0)     # Fallback if no response

ser.close()

signal_float = signal_clipped
filtered_float = np.array(filtered_signal, dtype=float)

with open(output_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(["Time", "Unfiltered", "Filtered"])
    for i in range(num_samples):
        writer.writerow([t[i], signal_float[i], filtered_float[i]])

print(f"Data saved to {output_csv}")