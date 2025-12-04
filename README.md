# ESC-RWPendulum

This repository contains the firmware, simulation models, and visualization tools for a **Reaction Wheel Pendulum** (Inverted Pendulum) control system. The project is designed to stabilize the pendulum using a reaction wheel driven by a DC motor, controlled by an ESP32 microcontroller.

## Project Structure

The repository is organized into three main directories:

* **`Firmware/`**: Embedded C++ code for the ESP32, built using PlatformIO. Handles sensor reading, control loop execution, and serial communication.
* **`Matlab/`**: MATLAB scripts and Simulink models for physical parameter definition, mathematical modeling, and control gain calculation (LQR).
* **`Visualization/`**: Python scripts for real-time telemetry, data logging, and plotting via Serial.

---

## 1. Firmware (ESP32)

The firmware is located in the `Firmware` directory and is configured as a **PlatformIO** project.

### Hardware Configuration
* **Microcontroller**: ESP32 (Board: `fm-devkit`)
* **Actuator**: DC Motor with PWM driver (controlled via pins 25/26).
* **Sensors**: Two Quadrature Encoders (Pendulum Angle and Wheel Speed).

**Pinout (`main.cpp`):**
| Component | Pin A | Pin B | Notes |
| :--- | :--- | :--- | :--- |
| **Wheel Encoder** | GPIO 17 | GPIO 16 | Measures reaction wheel speed/position |
| **Pendulum Encoder** | GPIO 19 | GPIO 18 | Measures pendulum tilt angle |
| **Motor Driver** | GPIO 25 | GPIO 26 | PWM Output (LEDC) |
| **Start/Stop Button**| GPIO 4 | - | Toggles control loop (ISR) |
| **Setup Button** | GPIO 34 | - | (Defined as `BOTAO_SU`, usage varies) |

### Control Loop
* **Algorithm**: State-Space Control (LQR).
* **Frequency**: 50 Hz (`Ts = 0.02s`).
* **States**:
    1.  $x_1$: Pendulum Angle (rad)
    2.  $x_2$: Wheel Angle (rad)
    3.  $x_3$: Pendulum Velocity (rad/s)
    4.  $x_4$: Wheel Velocity (rad/s)
* **Communication**: Sends state data and control effort via Serial (Binary format).

---
## 2. MATLAB & Simulation

The `Matlab` directory contains the mathematical foundation of the project.

* **`Scripts/SalavaParams.m`**: Defines the physical constants of the system (masses, lengths, inertia, motor constants) and saves them to `Parametros/parametros_planta.mat`.
    * *Parameters include:* $M_h$ (Hub mass), $M_r$ (Rotor mass), $L$ (Length), $J_h$ (Inertia), etc.
* **`PenduloRotativoSim.slx`**: Simulink model for simulating the system dynamics and controller response.
* **`Funcoes/`**: Contains auto-generated functions (`function_G_Rot`, `function_M_Rot`, etc.) for the non-linear simulation of the system.
* **`Parametros/`**: Stores `.mat` files with the calculated control gain matrix `K`.

**How to use:**
1.  Edit physical parameters in `SalavaParams.m` and run it.
2.  Run the main analysis script (e.g., `main.mlx`) to calculate the LQR gain matrix $K$.
3.  Update the `K` struct in `Firmware/src/main.cpp` with the calculated values.

---

## 3. Visualization (Python)

The `Visualization` directory provides tools to view system data in real-time.

### Requirements
* Python 3.x
* Libraries: `pyserial`, `matplotlib`

### Scripts
* **`ler_pendulo.py`**: A simple utility to read raw data from the serial port and print the decoded floating-point values to the console.
* **`plot_pendulo.py`**: A real-time plotting tool.
    * **Usage**: `python plot_pendulo.py --port COM3`
    * **Features**:
        * Plots multiple variables dynamically.
        * Supports a simulation mode for testing UI without hardware: `python plot_pendulo.py --sim`.
        * Visualizes Angle, Velocity, and Control Effort ($u$).

### Serial Protocol
The firmware sends a binary data packet structured as follows:
* **Start Byte**: `0xFF`
* **Payload**: 40 bytes (5 doubles, Little-Endian) representing:
    1.  $x_1$ (Pendulum Angle)
    2.  $x_2$ (Wheel Angle)
    3.  $x_3$ (Pendulum Velocity)
    4.  $x_4$ (Wheel Velocity)
    5.  $u$ (Control Voltage/Effort)
* **End Byte**: `0x00`

---

## License
*No license specified in repository files.*

## Author
**Gabriel Brunoro Motta Tumang**