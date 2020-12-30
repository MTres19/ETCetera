About
-----

ETCetera will be an electronic throttle controller program written for the
NuttX RTOS.

Building
--------

Clone the the upstream NuttX and NuttX apps repositories into two directories
called "nuttx" and "apps" respectively:

~~~
git clone https://github.com/apache/incubator-nuttx nuttx
git clone https://github.com/apache/incubator-nuttx-apps apps
~~~

Then, add ETCetera and ETCetera-tools as submodules:

~~~
cd apps/industry
git submodule add --branch main https://github.com/MTres19/ETCetera
git submodule add --branch main https://github.com/MTres19/ETCetera-tools
~~~

In the `nuttx` directory, use tools/configure.sh to select the
TM4C123G Launchpad NuttShell configuration:

~~~
cd ../../nuttx
./tools/configure.sh -l tm4c123g-launchpad:nsh
~~~

Next, change some Kconfig options with `make menuconfig`:

- Under **System Type → Options**:
  - Deselect **OABI** (use EABI instead)
  - Under **Toolchain Selection** select “Generic GNU EABI”
  - Under **Tiva/Stellaris Peripheral Support**:
    - Enable “ADC0”
    - Enable “CAN0”
    - (Future) Enable “PWM??”
- Under **Board Selection**
  - Change **CAN0 RX pin selection** to “Pin 4 on GPIO port B”
  - Change **CAN0 TX pin selection** to “Pin 5 on GPIO port B”
- Under **RTOS Features**:
  - Under **Clocks and Timers**:
    - Set **System timer tick period** to 1000.
  - Under **Tasks and Scheduling**:
    - Set **Application entry point** to “ETCetera_main”
  - Under **Work queue support**:
    - Enable “High priority (kernel) worker thread.” (This is required for the
      Tiva ADC driver.
- Under **Device Drivers**:
  - Enable “Analog Device Support”
  - Under **Analog Device Support**
    - Enable “Analog to Digital Conversion”
- Under **Application Configuration**:
  - Under **Industrial Applications**:
    - Enable ”ETCetera Throttle Controller”
    - Enable “Console utilities for the ETCetera Throttle controller”
  - Under **System Libraries and NSH Add-Ons**:
    - Make sure “NSH Example” is enabled

Save the configuration and exit Menuconfig. Then, build NuttX, ETCetera, and
ETCetera-tools by running `make`.

The ELF binary can be flashed by OpenOCD. You can either use OpenOCD directly,
or use the “load” command in a GDB session connected to OpenOCD.
