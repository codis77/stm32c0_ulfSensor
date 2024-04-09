# stm32c0_ulfSensor
a small sample application for the STM32C031C6, using the BMP280 pressure sensor

This application is a test implementation for the KY052 sensor board
with the BMP280 air pressure / temperature sensor, connected to the
STM32C031 Nucleo board.
The objective is to sample the air pressure at a 150Hz rate,
and transfer the sample data to the host for evaluation.

The code uses some ST Cube LL code, but not any of the bloated "HAL" code.
The Bosch BMP280 sensor is driven by SPI, and only raw pressure data
are collected. No compensation or calibration is applied, as they are
irrelevant for this purpose.

The code contains a calibration function.
If the user button is pressed during startup, the first batch of samples
is averaged, and sent to the host as calibration value.
Furthermore this calibration value is subtracted from all sample values.

