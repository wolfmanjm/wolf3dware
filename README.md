# wolf3dware
Prototype firmware for delta and cartesian printers on an STM32F4xxx

Heavily influenced by, and based on Smoothieware. Shares a great deal of code from that project.
https://github.com/Smoothieware/Smoothieware

Plans are to handle Inverse kinematics per step for a delta printer, with
improved Motion Control from SmoothieWare created by Arthur Wolf.

Uses the STM32F4xxx HAL by STM, there are no plans to make it portable to other architectures. However the HAL is abstracted with std::function calls.
Uses FreeRTOS as the kernel.

Initial prototype is running on an Olimex STM32-H405 https://www.olimex.com/Products/ARM/ST/STM32-H405/
With a STM32F405RGT6 

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
Test
