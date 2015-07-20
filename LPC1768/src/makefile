# Copyright 2015 Adam Green (http://mbed.org/users/AdamGreen/)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
PROJECT         := wolf3dware
DEVICES         := LPC1768
GCC4MBED_DIR    := /datadisk/aux/Stuff/reprap/gcc4mbed
NO_FLOAT_SCANF  := 1
NO_FLOAT_PRINTF := 1
MBED_LIBS       := rtos USBDevice
GCC4MBED_TYPE   := Debug
MRI_BREAK_ON_INIT := 1
MRI_UART        := MRI_UART_0
MRI_UART_BAUD   := 115200

include $(GCC4MBED_DIR)/build/gcc4mbed.mk

.PHONY: upload

upload: ./LPC1768/$(PROJECT).bin
        dfu-util -R -d 1d50:6015 -D $<


