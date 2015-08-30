# wolf3dware
Prototype firmware for delta and cartesian printers on an STM32F4xxx

Heavily influenced by, and based on Smoothieware. Shares a great deal of code from that project.
https://github.com/Smoothieware/Smoothieware

Plans are to handle Inverse kinematics per step for a delta printer, with
improved Motion Control from SmoothieWare created by Arthur Wolf.

Uses the STM32F4xxx HAL by STM, there are no plans to make it portable to other architectures. However the HAL is abstracted with std::function calls.
Uses FreeRTOS as the kernel.

Initial prototype is on a STM32F4-discovery board.


Install
-------
You need...

1. arm eabi gcc toolchain.. 
   a windows version is here http://gnutoolchains.com/arm-eabi/
   linux is here https://launchpad.net/gcc-arm-embedded

2. openocd and a JTAG adapter (I use the Bus Blaster)

3. Ruby and the Rake gem

4. set ARMTOOLS environment variabel to the arm tools you installed above

5. Then just type rake at the top level, and it will build.

There are various options that select which platform you are building for

Pins can be reassigned in Src/maincpp.cpp it is suggested you create a new #define for the target board




Status
------

* basic gcode parsing - done
* dispatch gcode to handlers - done
* motion control - done
* planner - done
* step generation - done
* temperature control - done
* LCD - done

Todo
----
Tested for motion
Needs more testing
