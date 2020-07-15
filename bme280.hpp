#ifndef _include_bme280_bme280_hpp_
#define _include_bme280_bme280_hpp_

#include <stdint.h>

namespace bme280
{
	extern bool DEBUG;

	class TBME280
	{
		protected:
			union calibration_t
			{
				struct
				{
					uint8_t part1[26];
					uint8_t part2[8];
				} __attribute__ ((packed));

				struct
				{
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
					int16_t  dig_H4;
					int16_t  dig_H5;
					int8_t   dig_H6;
				} __attribute__ ((packed));

				void ConvertEndianity();
			} __attribute__ ((packed));

			const int fd_i2cbus;
			const uint8_t address;
			calibration_t calibration;
			double temperature;
			double humidity;
			double pressure;

			void WriteRegister(const uint8_t index, const uint8_t value);
			uint8_t ReadRegister(const uint8_t index);
			void ReadRegister(const uint8_t index, const uint8_t n_bytes, void* const data);


		public:
			void Refresh();
			void Reset();

			TBME280(const char* const i2c_bus_device, const uint8_t address);
			TBME280(const int fd_i2cbus, const uint8_t address);
			~TBME280();
	};
}

#endif
