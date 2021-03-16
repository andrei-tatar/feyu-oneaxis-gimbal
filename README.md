# FeiyuTech WG2X (WG2 probably similar) Gimbal conversion to one axis

## Project goal
The goal of this project is to convert a WG2X 3 axis gimbal to a single axis gimbal that can keep a GoPro leveled on a FPV drone.

## Reverse engineering
I was only interested in the WG2X_C_v1.PcbDoc PCB, but the other 2 axis are very similar.

- MCU: **STM32F303CCT6** (72MHz, 256KB Flash, 40KB RAM)
  - 3V, 12MHz crystal
- BLDC controller: **AM2827**
  - STDBYA -> PB14
  - STDBYB -> PB15
  - STDBYC -> PB13
  - PWMA -> PA9
  - PWMB -> PA10
  - PWMC -> PA8
- HALL sensor: **MA730**
  - MISO -> PA6
  - MOSI -> PA7
  - SCK -> PA5
  - CS -> PA1
- IMU: **MPU6500**
  - SDO -> PB4
  - SDA/SDI -> PB5
  - SCL/SCK -> PB3
  - CS -> PB8
- SD06 opamp for current measurement
  - OUTA connected to PA0 (via 100 ohm)
  - OUTB connected to PA4 (via 100 ohm)
- Motor
  - 3 Phase **stepper** motor

## Firmware
In progress. PlatformIO.