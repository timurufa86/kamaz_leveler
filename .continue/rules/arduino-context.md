# Arduino Project Context

## Architecture

This is an Arduino project for [ваша плата, например: Arduino Uno / ESP32].

- Main sketch file: `main.ino`
- Custom libraries: `/lib`
- Pin definitions: defined in `config.h`

## Hardware

- Microcontroller: ATmega328P (Arduino Uno) [или ваш вариант]
- Available pins: 2-13 (digital), A0-A5 (analog)
- Memory: 32KB Flash, 2KB SRAM
- Serial: 9600 baud

## Coding Standards

- Use `const int` or `#define` for pin numbers
- Prefix pin definitions: `PIN_LED`, `PIN_BUTTON`
- Use `millis()` instead of `delay()` for non-blocking code
- Keep loop() fast and non-blocking
- Avoid dynamic memory allocation (no `new`, `malloc`)
- Use `F()` macro for Serial.print strings to save RAM

## Libraries Used

- [Перечислите ваши библиотеки, например: Adafruit_NeoPixel, DHT sensor library]

## External Documentation

- Arduino Language Reference: https://www.arduino.cc/reference/en/
- [Документация вашей платы]