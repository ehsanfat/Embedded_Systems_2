# Embedded_Systems_2
Programming the PIC controller- dsPIC33f

# Buggy Control 

This firmware is designed to control buggy, implementing various functionalities such as **motor control**, **state management**, **lights control**, **battery sensing**, **IR sensor reading**, and **UART** communication for data logging and command interface.

## Main Loop Requirements

-   **Control Loop Frequency:** The main control loop operates at 1 kHz.
-   **Motor PWM Update:** Motor PWM is updated at 1 kHz.
-   **IR Sensor Reading:** IR sensor is read at 1 kHz.
-   **Initial State:** The buggy starts in the "Wait for start" state with all motor PWM set to 0 and LED A0 blinking at 1 Hz.
-   **State Transition:** Upon pressing button RE8, the buggy transitions to the "Moving" state.
-   **LED Indicator:** LED A0 continues to blink at 1 Hz in both states.
-   **Return to Initial State:** Pressing button RE8 again returns the buggy to the "Wait for start" state.

## Motor Control

-   Surge and yaw_rate signals are used to compute left_pwm and right_pwm signals.
-   PWM signals are generated for motor control using Output Compare peripherals on pins RD1 to RD4.
-   PWM frequency is set to 10 kHz.
-   Actuation of wheels follows specified commands based on left_pwm and right_pwm signals.

## Lights Control

-   Buggy lights are controlled based on the current state.
-   Different light configurations are specified for the "Wait for start" and "Moving" states.

## Battery Sensing

-   Battery voltage is sensed using pin AN11.
-   Voltage is reported in Volt with a resolution of two digits (X.YZ) at 1 Hz frequency.

## IR Sensor

-   Infrared sensor signal is read on pin AN14.
-   Enable signal to the IR sensor is provided on digital I/O pin RB9.

## Data Logging / Command Interface through UART

-   UART to RS232 module is installed on the Clicker Mikrobus 2.
-   TX and RX signals are remapped to specific pins.
-   Messages are sent to the PC at specified frequencies, including battery voltage, sensed distance, and PWM duty cycles.
-   Messages are received from the PC, including threshold settings for MINTH and MAXTH.

## Note on UART Implementation

-   Proper dimensioning of buffers ensures no message loss due to UART implementation, even with full bandwidth utilization.

## Usage

1.  **Hardware Setup:** Connect the necessary components according to the hardware specifications provided.
2.  **Software Setup:** Upload the provided firmware to your microcontroller using the appropriate development environment.
3.  **Operation:**
    -   Press button RE8 to start the buggy and transition it to the "Moving" state.
    -   Press button RE8 again to return the buggy to the "Wait for start" state.
4.  **Monitoring and Control:** Use UART communication to monitor battery voltage, sensed distance, and adjust threshold settings as required.
