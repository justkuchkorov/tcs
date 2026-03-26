# Real-Time HIL Electric Vehicle Traction Control System

![Python](https://img.shields.io/badge/Python-3.x-blue?logo=python)
![CODESYS](https://img.shields.io/badge/CODESYS-V3.5-red)
![Protocol](https://img.shields.io/badge/Protocol-Modbus_TCP-green)

This repository contains the software implementation of a Hardware-in-the-Loop (HIL) Electric Vehicle Traction Control System (TCS). It was developed to validate closed-loop longitudinal slip control using an industrial SoftPLC and a real-time Python vehicle plant.

## System Architecture

The project is split into two distinct environments communicating over a local Modbus TCP/IP bridge at 50Hz (20ms cycle time).

1. **The Vehicle Plant (Python):** A 1D longitudinal dynamic model simulating a 1500kg RWD Electric Vehicle. It utilizes a Pacejka Magic Formula tire model for dry asphalt friction and calculates real-time wheel speed, vehicle speed, slip ratio, and dynamic axle load transfer.
2. **The Traction Controller (CODESYS PLC):** An industrial structured text program utilizing a time-synced PI controller (`PID_FIXCYCLE`) to regulate motor torque and maintain an optimal 15% slip ratio during aggressive launch scenarios.

## Key Control Features
* **Open-Loop Launch Override:** Bypasses closed-loop PI control below 3 m/s to prevent mathematical singularities and limit-cycle oscillations caused by simulated sensor quantization at standstill.
* **Slew-Rate Limiter:** Restricts torque transients to mathematically safe mechanical limits (80 Nm rise / 140 Nm fall per 20ms) to protect the physical drivetrain from instantaneous shock loading.
* **First-Order Low-Pass Filtering:** Smooths raw Modbus integer data to emulate physical wheel speed sensor noise reduction.
* **Anti-Hunting Deadband:** Freezes the PI integral term when the slip ratio is within ±2% of the target, eliminating steady-state high-frequency torque chatter.

## Repository Structure
* `/car-physics.py` - The real-time vehicle plant and Modbus TCP server.
* `/PLC_PRG.st` - The readable Structured Text source code for the CODESYS controller.
* **Releases Tab** - Contains the compiled binary CODESYS `.project` file for execution.

## How to Run the Simulation

### Prerequisites
* Python 3.x with `pyModbusTCP` and `matplotlib` installed.
* CODESYS V3.5 with the `Util` library installed.

### Execution Steps
1. Start the Modbus server and vehicle plant:
   ```bash
   python car-physics.py
2. Download the `tcs.project` file from the Releases tab and open it in CODESYS.
3. Ensure the Modbus TCP Target IP in CODESYS matches the Python host (e.g., 127.0.0.1 or your local Wi-Fi adapter IP).
4. Build -> Clean All, then press F11 to generate code.
5. Login to the virtual PLC and press Start.
6. Toggle the bStartTest boolean to TRUE to initiate the 3000 Nm launch sequence.
7. To generate telemetry graphs, toggle bStartTest to FALSE, wait 2 seconds, and press Ctrl+C in the Python terminal.

### Result
<img width="1920" height="1020" alt="image" src="https://github.com/user-attachments/assets/16d22164-e9d7-4d02-bb3f-0cf988a33c0a" />
