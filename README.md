About
-----

ETCetera will be an electronic throttle controller program
written for the NuttX RTOS.

Building
--------

Clone the the upstream NuttX and NuttX apps repositories into
two directories called "nuttx" and "apps" respectively. Then,
add this repository to apps as a submodule under the "industry"
folder.

Configure as usual with make menuconfig, but set ETCetera_main as
the entry point.
