# wolf3dware
Prototype firmware for delta printers on an STM32F4xxx

Heavily influenced by, and based on Smoothieware. Shares a great deal of code from that project.
https://github.com/Smoothieware/Smoothieware


Plans are to handle Inverse kinematics per step for a delta printer, with
improved Motion Control from SmoothieWare created by Arthur Wolf.

Will use the STM32F4xxx HAL by STM, there are no plans to make it portable to other architectures.

Initial prototype will be on a STM32F4-discovery board.

Status
------

basic gcode parsing - done
dispatch gcode to handlers - done
motion control - done
planner - done
step generation - done

Todo
----
port to STM32F
add timers for step generation
use FreeRTOS for threading
make block queue thread safe
