# RS41_gps_disciplined_oscillator

This small program for the STM32 microcontroller of the RS41 radiosonde configures the on-board GPS-receiver to generate a GPS synchronised 10MHz clock signal. The clock signal is present on pin 39 of the u-blox GPS-chip if the receiver manages to synchronize itself with the GPS-system. This clock signal could be used for a very stable local oscillator reference clock when receiving signals on the 10GHz band (for example Oscar 100). The crystal of a standard PLL-LNB could be replaced by the GPS disciplined oscillator built with a RS41 radiosonde. The stable 10MHz clock could also be used as reverence for clock generators like the SI5351 (yes it also works with 10MHz clocks).

I know that this system will never be as accurate as a real GPS disciplined oscillator but some sources on the internet state that it is possible to build clock generators with a stability of 0.01ppm @ 100MHz with u-blox GPS-receivers.

A cable driver should be added to run this with lab equipment which often has a 50 Ohm clock input. Also connecting long cables directly to the u-blox IC is not healty for the device.

This is a project for the µVision IDE. Feel free to port it to other IDEs.

You can see on the image where pin 39 is routed on the board. The outer shell of miniature coaxial cable is attached to the GND bellow the u-blox GPS chip.

[Pin 39 of the u-blox GPS receiver connection to coaxial cable](CLK_output.png)

More information about the RS41 hardware: https://github.com/bazjo/RS41_Hardware

## Flashing the radio probe

The hex files that must be programmed to the STM32 microcontroller can be found under:
    /MDK-ARM/GPS_clock/GPS_clock.hex

As programming tool a STLink is reaquired.
I recommend STM32CubeProgrammer for flashing. This program also allowes you to remove the read and write protection that is enabled when you get a fresh radio probe.

## Usage

* The device is turned on by pressing the power button once.
* The red LED will start glowing steadily. As soon the GPS receiver is outputing a clock, the red LED will start to flash.
* Power off is not implemented. Remove the power source to do so.

