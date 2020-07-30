#ifndef _include_bme280_bme280_hpp_
#define _include_bme280_bme280_hpp_

#include <stdint.h>

namespace bme280
{
	extern bool DEBUG;

	class TBME280
	{
		// this sensor tends to overheat... yes yes... a temperature sensor that tends to overheat... I know... don't ask
		// maybe cool it with ice so it can't overheat, how about it?
		// well actually it is NOT intented to be a temperature sensor, but it needs to be a temperature sensor
		// because it it needs to know the temperature to calculate the humidty and pressure
		// and it does A VERY POOR job at measuring the temperature, which means the temperature sensitive humidity reading
		// is mostly useless... but it can work as a makeshift pressure sensor...

		public:
			union calibration_t
			{
				// I have never seen a such ridiculously inconsistent data structure in my entire life. Thank you Bosch for this nonsense!

				struct
				{
					uint8_t part1[26];
					uint8_t part2[8];
				} __attribute__ ((packed));

				struct
				{
					// despite the fact that the entire chip always uses big-endian format the following fields are all little endian (*facepalm*)
					uint16_t dig_T1;
					int16_t  dig_T2;
					int16_t  dig_T3;

					uint16_t dig_P1;
					int16_t  dig_P2;
					int16_t  dig_P3;
					int16_t  dig_P4;
					int16_t  dig_P5;
					int16_t  dig_P6;
					int16_t  dig_P7;
					int16_t  dig_P8;
					int16_t  dig_P9;

					uint8_t  __reserved0;	//	addr 0xA0 not documented in datasheet

					uint8_t  dig_H1;

					// -------
					int16_t  dig_H2;
					uint8_t  dig_H3;

					int32_t  dig_H4 : 12,	// these two 12bit fields are in big-endian format (yey! Imagine how long it took to figure this out?)
							 dig_H5 : 12,
							 dig_H6 : 8;
				} __attribute__ ((packed));

				void ConvertEndianity();
			} __attribute__ ((packed));

		protected:
			const int fd_i2cbus;
			const uint8_t address;
			calibration_t calibration;

			void WriteRegister(const uint8_t index, const uint8_t value);
			uint8_t ReadRegister(const uint8_t index);
			void ReadRegister(const uint8_t index, const uint8_t n_bytes, void* const data);

		public:
			double temperature;
			double humidity;
			double pressure;

			void Refresh();
			void Reset();

			TBME280(const char* const i2c_bus_device, const uint8_t address);
			TBME280(const int fd_i2cbus, const uint8_t address);
			~TBME280();
	};
}

#endif
