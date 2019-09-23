# RS41_gps_disciplined_oscillator
This small program for the STM32 microcontroller of the RS41 radiosonde configures the on-board GPS-receiver to generate a GPS synchronised 10MHz clock signal. The clock signal is present on pin 39 of the GPS-chip if the receiver manages to synchronize itself with the GPS-system. This clock signal could be used for a very stable local oscillator reference clock when receiving signals on the 10GHz band (for example Oscar 100). The crystal of a standard PLL-LNB could be replaced by the GPS disciplined oscillator built with a RS41 radiosonde. The stable 10MHz clock could also be used as reverence for clock generators like the SI5351 (yes it also works with 10MHz clocks).

I know that this system will never be as accurate as a real GPS disciplined oscillator but some sources on the internet state that it is possible to build clock generators with a stability of 0.01ppm @ 100MHz with u-blox GPS-receivers.

The Drivers.zip must be extracted before opening the µVision project.

More information about the RS41 hardware: https://github.com/bazjo/RS41_Hardware

