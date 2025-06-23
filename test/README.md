# EZModbus Tests

## Overview

This test suite is designed for **PlatformIO with Arduino Core 3.0+** and uses custom testing frameworks optimized for that environment.

## Important Notice

⚠️ **These tests are NOT compatible with ESP-IDF native environment.**

The tests in this directory:
- Use PlatformIO-specific testing utilities
- Rely on Arduino Core abstractions
- Include custom mocking frameworks for the Arduino environment
- Are not designed to compile under ESP-IDF's CMake build system

## For ESP-IDF Users

If you're using EZModbus in an ESP-IDF project:
- Refer to the examples in `/examples/esp-idf/` instead
- The library itself is fully compatible with ESP-IDF
- Only these specific test files are PlatformIO/Arduino-only

## Running Tests

To run these tests:
1. Use PlatformIO with Arduino Core 3.0+
2. Setup your ESP32-S3 board in "loopback mode" with the following pin connections:
   ```
   RX1 (Pin 44) ←→ TX2 (Pin 43)
   TX1 (Pin 7)  ←→ RX2 (Pin 6)
   ```
3. Run with `pio test`

The core EZModbus library functionality remains identical across both Arduino and ESP-IDF environments.