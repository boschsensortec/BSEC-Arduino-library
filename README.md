# Instructions for using the BSEC Arduino Library in Arduino 1.8.13

## About BSEC

Bosch Sensortec Environmental Cluster (BSEC) Software v1.4.9.2 released on June 13th, 2022

The BSEC fusion library has been conceptualized to provide a higher-level signal processing and fusion for the BME680. The library receives compensated sensor values from the sensor API. It processes the BME680 signals to provide the requested sensor outputs.

Key features

- Precise calculation of ambient air temperature outside the device
- Precise calculation of ambient relative humidity outside the device
- Precise calculation of pressure outside the device
- Precise calculation of air quality (IAQ) level outside the device

Typical applications

- Health monitoring/ well-being (warning regarding dehydration / heat stroke)
- Home automation control
- Control heating, venting, air conditioning (HVAC) applications
- Gaming applications like flying toys
- Internet of things applications
- Context awareness
- Enhancement of GPS navigation (e.g., time-to-first-fix improvement, dead-reckoning, slope detection)
- Indoor navigation (floor detection, elevator detection)
- Outdoor navigation,
- Leisure and sports applications
- Weather forecast
- Health care applications (e.g. Spirometry)
- Vertical velocity indication (e.g. rise/sink speed)

Supported platforms

- BSEC library is supported on 32, 16 and 8 bit MCU platforms

Available binaries for download:

| Platform | Compiler | ROM (BSEC) | ROM (BSEC lite*) | RAM  | TYPE |
|----------|----------|------------|------------------|------|------|
| Cortex-ARM | ARMCC | 19-20k | 12-13k | 1k | Cortex-M0, M0+, M3, M4, M4_FPU, M7 |
| Cortex-ARM | GCC | 20-22k | 12-14k | 1k | Cortex-M0, M0+, M3, M4, M4_FPU, M7 |
| Cortex-ARM | IAR | 20k | 12-13k | 1k | Cortex-M0, M0+, M3, M4, M4_FPU, M7 |
| Cortex-A* | GCC | 21k | 13k | 1k | Cortex-A7 |
| AVR_8bit | AVR-GCC | 42k | 25 | 1k | MegaAVR, XMEGA |
| AVR_32bit | AVR-GCC | 24k | 13k | 1k | 32-bit AVR UC3 |
| ESP8266 | xtensa-lx106-elf-gcc | 28k | 17k | 1k | ESP8266 |
| ESP32 | xtensa-esp32-elf-gcc | 24k | 14k | 1k | ESP32 |
| MSP430 | msp430-elf-gcc | 34k | 20k | 1k | MSP430 |
| Android system-x86 | gcc | 39-49k | 22-26k | 1k | x86, x86_64 |
| Android system-arm | gcc | 21-38k | 13-19k | 1k | arm, arm64 |
| Raspberry PI 0 linux | arm-linux-gnueabihf-gcc | 71k | 56k | 1k | armv6-32bits |
| Raspberry PI3 linux | arm-linux-gnueabihf-gcc | 72k | 57k | 1k | armv8-a-64bits |

The library size information above doesn't include additional dependencies based on the embedded system project & platform.

*The BSEC lite version is an abbreviated version of BSEC with reduced code size & memory requirements. It does not include functions to save the state of BSEC, if the device powers down.

For other platforms, please contact your local Bosch Sensortec representative

Advantages

- Easy to integrate
- Hardware and software co-design for optimal performance
- Complete software fusion solution out of one hand
- Eliminates need for own fusion software development
- Robust virtual sensor outputs optimized for the application

## Software license agreement

The BSEC software is only available for download or use after accepting the software license agreement. By using this library, you have agreed to the terms of the license agreement.

[BSEC license agreement](https://www.bosch-sensortec.com/media/boschsensortec/downloads/bsec/2017-07-17_clickthrough_license_terms_environmentalib_sw_clean.pdf)

## Installation and getting started

### 1. Install the latest Arduino IDE

As of this publication, the latest Arduino IDE 1.8.13 can be downloaded from this [link](https://www.arduino.cc/download_handler.php)

### 2. Install the BSEC library

Either download this library as a zip and import it into the Arduino IDE. Refer to [this](https://www.arduino.cc/en/Guide/Libraries) guide on how to import libraries.

### 3. Modify the platform.txt file

If you have already used the previous example code and hack guide, remove the linker flag `-libalgobsec` in the platform.txt file and reference to the `compiler.c.elf.extra_flags`.

The standard arduino-builder now passes the linker flags under `compiler.libraries.ldflags`. Most platform.txt files do not already include this new optional variable. You will hence need to declare this variable's default and add it to the end of the combine recipe. It is recommended to declare it in the following section like below,

```
# These can be overridden in platform.local.txt
compiler.c.extra_flags=
compiler.c.elf.extra_flags=
#compiler.c.elf.extra_flags=-v
compiler.cpp.extra_flags=
compiler.S.extra_flags=
compiler.ar.extra_flags=
compiler.elf2hex.extra_flags=
compiler.libraries.ldflags=
```

and add it in the combine recipe like the below examples

#### ESP8266 community forum's ESP8266 core

Original line [105](https://github.com/esp8266/Arduino/blob/68ee1216454eeea49dd3452c6ff21bc748f397b6/platform.txt#L105),

```
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {build.exception_flags} -Wl,-Map "-Wl,{build.path}/{build.project_name}.map" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} -o "{build.path}/{build.project_name}.elf" -Wl,--start-group {object_files} "{archive_file_path}" {compiler.c.elf.libs} -Wl,--end-group  "-L{build.path}"
```

should become

```
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" {build.exception_flags} -Wl,-Map "-Wl,{build.path}/{build.project_name}.map" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} -o "{build.path}/{build.project_name}.elf" -Wl,--start-group {object_files} "{archive_file_path}" {compiler.c.elf.libs} {compiler.libraries.ldflags} -Wl,--end-group  "-L{build.path}"
```

#### Arduino's SAMD core

Original line [96](https://github.com/arduino/ArduinoCore-samd/blob/86081cbf35fc0df0612a1b2c054877ff6788f9e7/platform.txt#L96),

```
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}"  "-L{build.path}" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} "-T{build.variant.path}/{build.ldscript}" "-Wl,-Map,{build.path}/{build.project_name}.map" --specs=nano.specs --specs=nosys.specs {compiler.ldflags} -o "{build.path}/{build.project_name}.elf" {object_files} -Wl,--start-group {compiler.arm.cmsis.ldflags} -lm "{build.path}/{archive_file}" -Wl,--end-group
```

Should be,
```
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}"  "-L{build.path}" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} "-T{build.variant.path}/{build.ldscript}" "-Wl,-Map,{build.path}/{build.project_name}.map" --specs=nano.specs --specs=nosys.specs {compiler.ldflags} -o "{build.path}/{build.project_name}.elf" {object_files} -Wl,--start-group {compiler.arm.cmsis.ldflags} -lm "{build.path}/{archive_file}" {compiler.libraries.ldflags} -Wl,--end-group
```

### 4. Verify and upload the example code

Start or restart the Arduino IDE. Open the example code found under ```File>Examples>Bsec software library>Basic```.

Select your board and COM port. Upload the example. Open the Serial monitor. You should see an output on the terminal.

Note that not all supported cores have been tested. In such cases, the examples can be found under ```File>Examples>INCOMPATIBLE>Bsec software library>Basic```

### 5. Tested board/core list

The current list of tested boards include,

| Core MCU | Tested boards | Arduino core version | Arduino core repository |
|----------|---------------|----------------------|-------------------------|
| Atmega 2560 | Arduino MEGA 2560 | Shipped with Arduino 1.8.13 | Shipped with Arduino 1.8.13 |
| Cortex-m0+ | Arduino Zero | Upstream of v1.6.21 SHA-1 hash 86081cbf35fc0df0612a1b2c054877ff6788f9e7 | https://github.com/arduino/ArduinoCore-samd |
| Cortex-m3 | Arduino Due | SHA-1 hash 0a4c3b196a02e48e31b752a05d8c8064007874dc | https://github.com/arduino/ArduinoCore-sam |
| Cortex-m4 with FPU | Adafruit BlueFruit NRF52 Feather | Upstream of v0.10.1 SHA-1 hash 11614dae701a35f905d09792c7388d648b125369 | https://github.com/adafruit/Adafruit_nRF52_Arduino |
| Esp32 | Sparkfun ESP32 Thing | v1.0.3-rc1 | https://github.com/espressif/arduino-esp32 |
| Esp8266 | Adafruit Feather HUZZAH | Upstream of v2.5.1 SHA-1 hash 625c3a62c4991347e8298fb5e4021bc6f6df7099 | https://github.com/esp8266/Arduino |

## Copyright (C) 2020 Bosch Sensortec GmbH. All rights reserved.
