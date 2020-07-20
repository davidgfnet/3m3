
3m3 - Triple GSM modem
======================

WTF is this?

3m3 is a small hardware project to build a device that holds three GSM modems
(SIM800L specifically) and is connected to a computer via USB using standard
software interface. The goal is to be able to receive and send SMS in a more
scalable way than just buying USB LTE modems and USB hubs :)

Initially the intent was to host 4 modems but that requires more powerful
STM32 devices. STM32F407 does not fit the budget due to some restrictions on
the USB interface (FS is limited to 6 USB EPs).


