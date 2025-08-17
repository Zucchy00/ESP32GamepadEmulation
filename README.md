# premise
Similar to [ESP32GamepadEmulation](https://github.com/Zucchy00/ESP32GamepadEmulation) this work's with the same logic however this aims to be lighter as code as this will only emulate ps4 controller to add the only missing buttons. 
## PsVitaCustomController
### WORK-IN PROGRESS
Esp 32 based ps vita controller using ESP-IDF
### Any contributions is appreciated
### How works:
Using [ds34Vita](https://github.com/MERLev/ds34vita) as driver i want to emulate a bluetooth controller with bluetooth classic to add R2, L2, R3 and L3 to the ps vita without modifying the hardware of the console

## Improvements

* Steam Input recognized the controller as a ps4 one ( however it needs to handle also the events and features request of it to correctly being recognized as one from every driver )
* Original ps4 controller hid descriptor doesn't crash anymore ( does not connect but not boot loop )
