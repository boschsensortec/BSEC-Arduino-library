# Instructions for using the BSEC Arduino Library in Arduino 1.8.8

The following steps/hacks will need to be followed to integrate the BSEC library into your project.

## Installation and getting started

### 1. Install the latest Arduino IDE

As of this publication, the latest Arduino IDE 1.8.8 can be downloaded from this [link](https://www.arduino.cc/download_handler.php)

### 2. Install the BSEC library

Either download this library as a zip and import it into the Arduino IDE. Refer to [this](https://www.arduino.cc/en/Guide/Libraries) guide on how to import libraries.

### 3. Modify the platform.txt file

If you have already used the previous example code remove the linker flag `-libalgobsec` in the platform.txt file.

The arduino-builder passes the linker flags under `{compiler.libraries.ldflags}`. Open you platform.txt and add this near the end of your recipe.c.combine.pattern.

#### Examples

##### ESP8266 community forum's ESP8266 core v2.5.0-beta2

Original line [103, 104](https://github.com/esp8266/Arduino/blob/0fd86a07f0b22aa43f24dc6158b6d8093b607765/platform.txt#L103),

```
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" -Wl,-Map "-Wl,{build.path}/{build.project_name}.map" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} -o "{build.path}/{build.project_name}.elf" -Wl,--start-group {object_files} "{archive_file_path}" {compiler.c.elf.libs} -Wl,--end-group  "-L{build.path}"
```

should become

```
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}" -Wl,-Map "-Wl,{build.path}/{build.project_name}.map" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} -o "{build.path}/{build.project_name}.elf" -Wl,--start-group {object_files} "{archive_file_path}" {compiler.c.elf.libs} {compiler.libraries.ldflags} -Wl,--end-group  "-L{build.path}"
```

##### Arduino's SAMD core v1.6.1

Original line [91, 92](https://github.com/arduino/ArduinoCore-samd/blob/ed40dd839eb6aa84b5d01ba16db57b687d99239d/platform.txt#L91),

```
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}"  "-L{build.path}" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} "-T{build.variant.path}/{build.ldscript}" "-Wl,-Map,{build.path}/{build.project_name}.map" --specs=nano.specs --specs=nosys.specs {compiler.ldflags} -o "{build.path}/{build.project_name}.elf" {object_files} -Wl,--start-group -lm "{build.path}/{archive_file}" -Wl,--end-group
```

Should be,
```
## Combine gc-sections, archives, and objects
recipe.c.combine.pattern="{compiler.path}{compiler.c.elf.cmd}"  "-L{build.path}" {compiler.c.elf.flags} {compiler.c.elf.extra_flags} "-T{build.variant.path}/{build.ldscript}" "-Wl,-Map,{build.path}/{build.project_name}.map" --specs=nano.specs --specs=nosys.specs {compiler.ldflags} -o "{build.path}/{build.project_name}.elf" {object_files} -Wl,--start-group -lm {compiler.libraries.ldflags} "{build.path}/{archive_file}" -Wl,--end-group
```

### 4. Only for the ESP8266 - modify the linker script

Due to the current size of the BSEC library, upon compilation, you will receive an error: ```section `.text' will not fit in region `iram1_0_seg'```. In order to solve this, you will need to modify the source definition of the linker script and specifically define where the library should be placed in memory.

In previous versions of this manual it was stated that you need to edit `eagle.app.v6.common.ld` directly. Since this file is now generated at build time you will need to change the linker script file, `eagle.app.v6.common.ld.h` typically found in `{YourESP8266PPackageDirectory}\tools\sdk\ld`. 

With reference to the linker script [here](https://github.com/esp8266/Arduino/blob/0fd86a07f0b22aa43f24dc6158b6d8093b607765/tools/sdk/ld/eagle.app.v6.common.ld.h#L138),

After line 138, add *libalgobsec.a:(.literal.* .text.*), which should look like,

```
    *libupgrade.a:(.literal.* .text.*)
    *libwpa.a:(.literal.* .text.*)
    *libwpa2.a:(.literal.* .text.*)
    *libwps.a:(.literal.* .text.*)
	*libalgobsec.a:(.literal.* .text.*)
    *(.irom0.literal .irom.literal .irom.text.literal .irom0.text .irom0.text.* .irom.text .irom.text.*)

    /* __FUNCTION__ locals */
    *(.rodata._ZZ*__FUNCTION__)
```

### 5. Copy the binaries

If you have already used the previous example code remove the `libalgobsec.a` file from the core directory or any other location you might have copied it to and instead, copy the binaries from the zip file available via our [website](https://www.bosch-sensortec.com/en/bst/products/all_products/bsec), to where the Arduino library is installed on your system. 
For Windows, there are three typical locations where the library might be imported.
- `Documents/Arduino/libraries/bsec`
- `C:\Program Files (x86)\Arduino\libraries\bsec`
- `<Sketchbook location>\libraries\bsec`
The library name might differ depending on how you installed it, e.g. bsec, BSEC-Arduino-library-master or similar. Find your <Sketchbook location> by opening Arduino IDE and going to ```File>Preferences```. If the library is imported correctly, you will see multiple subfolders containing text files.

| From the .zip (algo/bin/Normal_version/) | To (<bsec-lib>/src/) |
|------|----|
| avr/AVR8_megaAVR | atmega2560 |
| gcc/Cortex_M0+ | cortex-m0plus |
| gcc/Cortex_M3 | cortex-m3 |
| gcc/Cortex_M4F | cortex-m4 |
| gcc/Cortex_M4F | cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard |
| esp32 |  esp32 |
| esp8266 |  esp8266 |

### 6. Verify and upload the example code

Start or restart the Arduino IDE. Open the example code found under ```File>Examples>BSEC Software Library>basic```.

Select your board and COM port. Upload the example. Open the Serial monitor. You should see an output on the terminal.

Note that not all supported cores have been tested. In such cases, the examples can be found under ```File>Examples>INCOMPATIBLE>Bsec software library>Basic```

### 7. Tested board list

The current list of tested micro-controllers include,

| Core MCU | Tested boards |
|----------|---------------|
| atmega2560 | Arduino MEGA 2560 |
| cortex-m0plus | Arduino Zero |
| cortex-m3 | Arduino Due |
| cortex-m4f | Adafruit BlueFruit NRF52 Feather, STM32 Nucleo F411RE |
| esp32 | Sparkfun ESP32 Thing |
| esp8266 | Adafruit Feather HUZZAH |
## Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
