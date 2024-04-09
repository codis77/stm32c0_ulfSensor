# stm32c0_ulfSensor
-- 8.4.2024
a small sample application for the STM32C031C6, using the BMP280 pressure sensor

This application is a test implementation for the KY052 sensor board
with the BMP280 air pressure / temperature sensor.
The objective is to sample the air pressure at 150Hz sampling rate,
and transfer the sample data to the host for evaluation.

The BMP280/KY052 board is interfaced via SPI.

Initial version used direct register access, but later ones had some
Cube_LL code added in. The ugly STM Cube HAL code is out of question, though.

The project is set up for Segger Embedded Studio.
All required source file are included, although loading the respactive
support packages in SES is probably requiered.
