# bme280-driver
A compact driver for the Bosch BME280 temperature, humidiy and barometric pressure sensor chip written in C++.

### Personal Remarks
This chip is a nightmare!
While it is accurate and works reliably, it is a nightmare to interface with.
Wild mixing of big-/little endian. Weird register layout. Bit-fields everywhere. Calibration registers without documentation.
The Bosch Library is no help here too, since it doesn't even compile! (at least not without putting some work into it).
A tiny oversight (some calibration registers are encoded in big-endian, while most are encoded little-endian) can lead to reasonably valid looking temperature values, which just happen to be off by 7°C or so.
