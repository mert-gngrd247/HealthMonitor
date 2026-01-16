import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import collections

# --- CONFIGURATION ---
SERIAL_PORT = 'COM9' 
BAUD_RATE = 115200

# --- DATA STORAGE ---
# We keep the last 50 data points for a rolling window
MAX_POINTS = 50
data_temp = collections.deque([0] * MAX_POINTS, maxlen=MAX_POINTS)
data_volt = collections.deque([0] * MAX_POINTS, maxlen=MAX_POINTS)

# --- SETUP SERIAL ---
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except:
    print(f"ERROR: Could not open {SERIAL_PORT}. Is the port correct? Is another app using it?")
    exit()

# --- PLOT SETUP ---
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)
fig.suptitle('STM32 Health Monitor')

def update(frame):
    try:
        # 1. Read a line from the STM32
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            
            # 2. Parse CSV (Voltage,Temperature)
            parts = line.split(',')
            if len(parts) == 2:
                voltage_mv = int(parts[0])
                temp_c = int(parts[1])
                
                # 3. Store Data
                data_volt.append(voltage_mv)
                data_temp.append(temp_c)
                
                # 4. Refresh Plots
                ax1.cla()
                ax1.set_ylabel('Voltage (mV)')
                ax1.plot(data_volt, color='blue', label='VDDA')
                ax1.legend(loc='upper left')
                ax1.set_ylim(3200, 3400) # Zoom in on 3.3V range

                ax2.cla()
                ax2.set_ylabel('Temp (°C)')
                ax2.plot(data_temp, color='red', label='Core Temp')
                ax2.legend(loc='upper left')
                ax2.set_ylim(20, 45) # Typical running range

    except ValueError:
        pass # Ignore bad data lines
    except Exception as e:
        print(f"Error: {e}")

# --- RUN ANIMATION ---
ani = FuncAnimation(fig, update, interval=100) # Update every 100ms
plt.show()

# Close serial on exit
ser.close()

