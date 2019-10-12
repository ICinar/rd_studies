This is a description how to port FreeRTOS to Atmel SAM R21 

1. Change to SAMR21 directory:  `cd SAMR21_FreeRTOS_Clockdrift/FreeRTOS`

2. To create elf file for flashing:  `make all`
3. To flash with openocd to SAM R21: `openocd -f samR21.cfg -c "program FREERTOS_CLOCKDRIFT.elf" verify reset exit`
