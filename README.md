# PHOBOS
The larger and innermost of the two natural satellites of Mars, Phobos is the
embedded system that controls the static (fixed-base) bicycle simulator and
steer-by-wire bicycle.

## Toolchain
To compile the code, use [GNU ARM Embedded
toolchain](https://launchpad.net/gcc-arm-embedded).
The version used is 5.4-2016-q2.

Some examples may not build as the option `-fsingle-precision-constant` is not
standard compliant. The issue is documented
[here](https://bugs.launchpad.net/gcc-arm-embedded/+bug/1452470) and contains a
fix.

As most computers will be running a 64-bit kernel, libraries for ia32/i386
architecture will need to be installed if not already. For recent versions of
Ubuntu, this can be installed with the `gcc-multilib` package.

## Building
As this project is dependent on a number of git submodules, as well as nested
submodules, they should be initialized when the repository is first cloned.

    oliver@canopus:~/repos/phobos$ git submodule update --init --recursive --recommend-shallow

When creating the CMake build tree, the compiler must be set to the target
toolchain to allow cross-compilation.  CMake cannot change the compiler after
the build tree has been created and if the toolchain is not provided, the
default compiler will be used.  The path to the toolchain must be defined in a
CMake script file and passed as in the argument `CMAKE_TOOLCHAIN_FILE` the first
time `cmake` is called.  An example file is provided in the root project
directory.

    oliver@canopus:~/repos/phobos$ mkdir build
    oliver@canopus:~/repos/phobos$ cd build/
    oliver@canopus:~/repos/phobos/build$ cmake -DCMAKE_TOOLCHAIN_FILE=~/toolchain/toolchain-gcc-arm-none-eabi-5_4-2016q2.cmake ..
    oliver@canopus:~/repos/phobos/build$ make -j8

Build options can be passed as arguments to `cmake` or set with `ccmake` after
the build tree has been created.

    oliver@canopus:~/repos/phobos/build$ ccmake .

## Flashing
The compiled ELF can be flashed to the microcontroller by enabling the
`OPENOCD_FLASH_TARGET` or `OPENOCD_FLASH_AND_RUN_TARGET` CMake options.

If you are running OSX and unable to flash successfully with CMake, OpenOCD may
be requesting to receive incoming network connections. This can be done by
signing the application:

    oliver@canopus:~$ codesign --verify -vv /usr/local/Cellar/open-ocd/HEAD/bin/openocd

In 'Security & Privacy', verify that the option 'Automatically allow signed
software to receive incoming connections' is enabled.

## Running
Most of the demos blink the board LED when running. Most demos also send ASCII
over serial and increase the blink rate of the LED when a USB connection is
established. The exception is the FATFS demo as use the SDIO uses a same pin as
USB so both peripherals cannot be used at once. The transmitted characters can
be viewed using any serial terminal program. Here's an example using screen

    oliver@canopus:~$ screen /dev/tty.usbmodem311

As a virtual serial port is established, baud rate is ignored and has no impact
on USB transfer speed. The serial data can also be logged to file with screen

    oliver@canopus:~$ screen -L /dev/tty.usbmodem311

Using screen to log serial data to file can occasionally result in missing
bytes. In this case, `seriallog` in the tools directory can be used to log to
file.

Some projects output serial data just as bytes (not ASCII). Cutecom can be used
to view the data as hex. You can download it
[here](http://cutecom.sourceforge.net/). If you used homebrew to install Qt4,
Qt3 support will be missing. You can reinstall Qt4 with the following option to
enable support

    oliver@canopus:~$ brew reinstall qt --with-qt3support

The projects in this repository are designed to run on an [Olimex
STM32-H405](https://www.olimex.com/Products/ARM/ST/STM32-H405/) microcontroller
together with a set of [PCBs](https://github.com/oliverlee/gyropcb)
([v0.1](https://github.com/oliverlee/gyropcb/releases)) in order to interface
with a number of sensors and actuators. When using the PCBs, please make sure to
check the errata in the project README.

*Windows users may need to install STMicroelectronics Virtual COM Port driver
1.3.1.*

## Debugging
The microcontroller can be debugged with OpenOCD and GDB.

    oliver@canopus:~/repos/phobos$ openocd -f openocd_olimex-arm-usb-tiny-h_stm32f4.cfg

In another shell instance, pass GDB the ELF as an argument

    oliver@canopus:~/repos/phobos/build$ ~/toolchain/gcc-arm-none-eabi-5_4-2016q2/bin/arm-none-eabi-gdb-py demos/usb_serial/usb-serial.elf

To connect to target

    (gdb) target extended-remote localhost:3333

Additionally, the ELF can be flashed

    (gdb) load
    (gdb) mon reset init

There is an example `gdbinit` file in the root directory that defines a command
to connect to the remote and set hardware break/watch point limits in addition
to disabling IRQs while stepping. This can be used by renaming the file to
`.gdbinit` and placing in your home or project directory or by executing GDB
commands from a file when starting GDB

    oliver@canopus:~/repos/phobos/build$ ~/toolchain/gcc-arm-none-eabi-5_4-2016q2/bin/arm-none-eabi-gdb-py -x ../gdbinit demos/usb_serial/usb-serial.elf

If the sample gdbinit file is used, the gdb binary `arm-none-eabi-gdb-py` (with
python scripting enabled) must also be used.

In some cases when debugging, Link Time Optimization will remove necessary debug
information (such as Eigen Matrix template arguments). You may wish to disable
LTO (CMake option `CHIBIOS_USE_LTO`) in these cases.

## Debugging with Eclipse
Eclipse can also be used for debugging and can be downloaded
[here](https://www.eclipse.org/downloads/). With the use of
plugins, it is much easier to read register values and view thread status and
stack usage.

For general instructions refer to the ChibiOS/Eclipse guides [part
1](http://www.chibios.org/dokuwiki/doku.php?id=chibios:guides:eclipse1) and
[part 2](http://www.chibios.org/dokuwiki/doku.php?id=chibios:guides:eclipse2).

### Project Setup
- Install the Eclipse IDE for C/C++ Developers. You probably need the JDK which
  can be found
  [here](http://www.oracle.com/technetwork/java/javase/downloads/jdk8-downloads-2133151.html).
- Create an Eclipse project with CMake: (don't include the linebreak)  
  `oliver@canopus:~/repos/phobos/build-eclipse$ cmake -G"Eclipse CDT4 - Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=../toolchain-gcc-arm-none-eabi-5_4-2016q2.cmake ..`  
  Don't forget to set build type to 'Debug' and enable ChibiOS debug options.
- In Eclipse, import the project using 'File->Import'.  
  Select 'General->Existing Projects into Workspace'.  
  Set 'Select root directory:' to the previously used build directory and select the project.
- In 'Project->Properties', 'C/C++ Make Project->Binary Parser', select 'GNU Elf
  Parser'.
- OpenOCD can be configured as an external tool.  
  Provide the location of the binary, set the working directory to be  
  `${workspace_loc:/PHOBOS-Debug@build-eclipse}` and set arguments to  
  `-f ../openocd_olimex-arm-usb-tiny-h_stm32f4.cfg`

### Plugin Setup
- 'C/C++ GDB Hardware Debugging' can be found in the 'Mobile and Device
  Development' category of available software.
- 'EmbSysRegView' can be found in 'Help->Eclipse Marketplace'.
- Copy ChibiOS jar files found in eclipse/dropins to the Eclipse dropins
  directory. For OSX, this is located at
  `/Applications/Eclipse.app/Contents/Eclipse/dropins/`

## Demo and Project descriptions
Most demos and projects will print ASCII output using serial over USB and blink
the LED to denote that the code is running. A brief description of each
executable is given below:

    +-- demos
    |   +-- adc             - Reads ADC channels ADC10, ADC11, ADC12.
    |   +-- bicycle         - Simulates the Whipple model with Kalman filter and prints state and auxiliary state.
    |   +-- dac             - Outputs new DAC values at 1 kHz with large steps between each value.
    |   +-- eigen           - Performs matrix multiplication and matrix element rescaling.
    |   +-- encoder         - Activates incremental encoder with 15200 counts per revolution.
    |   +-- encoder_hots    - Activates incremental encoder with 15200 counts per revolution using Higher-Order Time-Sampling.
    |   +-- encoder_index   - Activates incremental encoder with 15200 counts per revolution and index channel.
    |   +-- fatfs           - Writes and reads data to file on micro SD card.
    |   +-- gpt_adc         - Reads ADC channels ADC10, ADC11, ADC12 and uses GPT triggering.
    |   +-- pwm_encoder     - Activates 2 PWM channels 1/4 period of of phase and an incremental encoder.
    |   +-- usb_serial      - Runs ChibiOS usb-serial demo and runs tests.
    |
    +-- projects
        +-- clustril        - Runs development static simulator code. Requires usage of SDIO which prevents usage of serial over USB.
        +-- drunlo          - Prints sensor values as ASCII. Requires usage of serial over USB.
        +-- flimnap         - Runs static simulator code interfacing with Unity environment Bikesim(5ade352 or newer). Cannot log data via SDIO.
