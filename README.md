# nrf52832_joystick
this is a simple bluetooth joystick using NRF52832.
Basically it works ok although I am not quite satisfied with the performance.
The only issue I am seeing at the moment is ble_hids_inp_rep_send() reports NRF_ERROR_RESOURCES
quite offten without the delay call. I'll be digging it farther later.

The basc joystick interface board scehmatic can be found [here](https://github.com/peakhunt/stm32f103c8t6_usb_hid)

The NRF module I used is something called gold core. It's just a cheap chinese NRF52 module.

![Front][captures/front.jpg "front")
![Back1][captures/back1.jpg "back1")
![Back2][captures/back1.jpg "back2")
![Back3][captures/back1.jpg "back3")
