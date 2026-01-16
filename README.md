# STM32 System Health Monitor (ADC + DMA)

A robust, non-blocking background monitoring system for the STM32G031K8. This project continuously acquires internal temperature and voltage reference data using **Direct Memory Access (DMA)**, applies factory calibration for high precision, and executes closed-loop thermal protection logic.

## 🎯 Project Overview
In safety-critical embedded systems, blocking the CPU to read sensors is unacceptable. This project demonstrates how to offload data acquisition to the hardware (DMA), allowing the CPU to focus solely on processing and control logic.

* **Zero-CPU Acquisition:** Uses `ADC_Circular` mode to fill memory buffers automatically.
* **Precision Engineering:** Implements dynamic voltage compensation to correct raw ADC readings based on actual $V_{DDA}$ supply drift.
* **Factory Calibration:** Utilizes ST's unique factory-trimmed memory addresses to convert raw sensor data into accurate engineering units (°C and mV).
* **Safety Logic:** "Thermostat" feature triggers a hardware alarm (LED) when the die temperature exceeds a defined threshold (31°C).

## 🛠 Hardware Configuration
* **Microcontroller:** STM32G031K8 (Nucleo-32 Board)
* **Sensors (Internal):**
    * `ADC_IN12`: Internal Temperature Sensor.
    * `ADC_IN13`: Internal Voltage Reference ($V_{REFINT}$).
* **Actuator:** On-Board LED (Green) on **PC6**.

## ⚙️ Technical Implementation

### 1. DMA & Circular Buffers
Instead of polling `HAL_ADC_GetValue()`, the system configures the DMA controller to continuously transfer conversion results into a global array `adc_buffer[2]`.
```c
// ADC starts once, DMA manages the data transfer forever
HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 2);

### 2. Voltage Compensation
Raw ADC values on the STM32 depend on the supply voltage (VDDA). Since USB power can fluctuate (e.g., 3.28V vs 3.3V), a raw reading of "932" is ambiguous. The system calculates the *actual* supply voltage first to normalize the sensor data:

`VDDA = 3000 mV * (VREFINT_CAL / VREFINT_DATA)`

### 3. Temperature Calculation (Linear Interpolation)
The code retrieves the factory calibration constants stored in the System Memory (OTP area) during manufacturing:

* `TS_CAL1`: Raw ADC value at 30°C (3.0V)
* `TS_CAL2`: Raw ADC value at 130°C (3.0V)

The final temperature is computed by interpolating the voltage-compensated raw value between these two calibration points.

## 🚀 How to Run
1. Open the project in **STM32CubeIDE**.
2. Build and Flash to the **Nucleo-G031K8** board.
3. Open a Serial Terminal (115200 baud, 8-N-1).
4. **Observe:**
   * Real-time Voltage and Temperature logs.
   * **Test:** Place your finger on the MCU to raise the temp > 31°C. The Green LED will light up.

## 📂 File Structure
* `Core/Src/main.c`: Contains the Primary Control Loop, Math logic, and Hardware Initialization.
* `Core/Inc/stm32g0xx_hal_conf.h`: HAL Configuration.
* `HealthMonitor.ioc`: STM32CubeMX Configuration file.

## 📊 Python Visualization (HIL Dashboard)
This project includes a Python script to visualize the real-time telemetry stream.

### Features
* **Live Plotting:** Displays Voltage (mV) and Temperature (°C) on dual dynamic graphs.
* **Auto-Scaling:** Automatically adjusts Y-axis based on incoming data range.
* **CSV Parsing:** Decodes the comma-separated UART stream from the STM32.

### Usage
1. Navigate to the `scripts/` directory.
2. Install dependencies: `pip install -r requirements.txt`
3. Connect the Nucleo board via USB.
4. Run the dashboard: `python dashboard.py`



