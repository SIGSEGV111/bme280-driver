#include <sys/uio.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <endian.h>
#include "bme280.hpp"

#define SYSERR(expr) (([&](){ const auto r = ((expr)); if( (long)r == -1L ) { throw #expr; } else return r; })())

namespace bosch
{
	// The code below is (mostly) not from me. I claim no responsibility for the mess! This came straight from Bosch from the BME280 datasheet.
	// I only did minor modifications to make it find all input values.
	// I also have no clue how any person ever thought that a Q24.8 or Q22.10 format was a good idea.

	// ---------------------------------------

	// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC.
	// t_fine carries fine temperature as global value
	static int32_t BME280_compensate_T_int32(const int32_t adc_T, int32_t& t_fine, const bme280::TBME280::calibration_t& cal)
	{
		int32_t var1, var2, T;
		var1 = ((((adc_T>>3) - ((int32_t)cal.dig_T1<<1))) * ((int32_t)cal.dig_T2)) >> 11;
		var2 = (((((adc_T>>4) - ((int32_t)cal.dig_T1)) * ((adc_T>>4) - ((int32_t)cal.dig_T1))) >> 12) * ((int32_t)cal.dig_T3)) >> 14;
		t_fine = var1 + var2;
		T = (t_fine * 5 + 128) >> 8;
		return T;
	}

	// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
	// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
	static uint32_t BME280_compensate_P_int64(int32_t adc_P, const int32_t t_fine, const bme280::TBME280::calibration_t& cal)
	{
		int64_t var1, var2, p;
		var1 = ((int64_t)t_fine) - 128000;
		var2 = var1 * var1 * (int64_t)cal.dig_P6;
		var2 = var2 + ((var1*(int64_t)cal.dig_P5)<<17);
		var2 = var2 + (((int64_t)cal.dig_P4)<<35);
		var1 = ((var1 * var1 * (int64_t)cal.dig_P3)>>8) + ((var1 * (int64_t)cal.dig_P2)<<12);
		var1 = (((((int64_t)1)<<47)+var1))*((int64_t)cal.dig_P1)>>33;
		if (var1 == 0)
		{
			return 0; // avoid exception caused by division by zero
		}
		p = 1048576-adc_P;
		p = (((p<<31)-var2)*3125)/var1;
		var1 = (((int64_t)cal.dig_P9) * (p>>13) * (p>>13)) >> 25;
		var2 = (((int64_t)cal.dig_P8) * p) >> 19;
		p = ((p + var1 + var2) >> 8) + (((int64_t)cal.dig_P7)<<4);
		return (uint32_t)p;
	}

	// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
	// Output value of "47445" represents 47445/1024 = 46.333 %RH
	static uint32_t bme280_compensate_H_int32(int32_t adc_H, const int32_t t_fine, const bme280::TBME280::calibration_t& cal)
	{
		int32_t v_x1_u32r;
		v_x1_u32r = (t_fine - ((int32_t)76800));
		v_x1_u32r = (((((adc_H << 14) - (((int32_t)cal.dig_H4) << 20) - (((int32_t)cal.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)cal.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)cal.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)cal.dig_H2) + 8192) >> 14));
		v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)cal.dig_H1)) >> 4));
		v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
		v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
		return (uint32_t)(v_x1_u32r>>12);
	}

	// ---------------------------------------
}

namespace bme280
{
	bool DEBUG = false;

	void TBME280::calibration_t::ConvertEndianity()
	{
		dig_T1 = le16toh(dig_T1);
		dig_T2 = le16toh(dig_T2);
		dig_T3 = le16toh(dig_T3);
		dig_P1 = le16toh(dig_P1);
		dig_P2 = le16toh(dig_P2);
		dig_P3 = le16toh(dig_P3);
		dig_P4 = le16toh(dig_P4);
		dig_P5 = le16toh(dig_P5);
		dig_P6 = le16toh(dig_P6);
		dig_P7 = le16toh(dig_P7);
		dig_P8 = le16toh(dig_P8);
		dig_P9 = le16toh(dig_P9);
		dig_H2 = le16toh(dig_H2);

		// something tells me this might not do what I think it should do... but the humidity values look good now
		dig_H4 = be16toh((uint16_t)dig_H4);
		dig_H5 = be16toh((uint16_t)dig_H5);
	}

	void TBME280::WriteRegister(const uint8_t index, const uint8_t value)
	{
		uint8_t buffer[2];
		buffer[0] = index;
		buffer[1] = value;

		if(SYSERR(write(this->fd_i2cbus, buffer, 2)) != 2)
			throw "write register failed (8bit)";
	}

	uint8_t TBME280::ReadRegister(const uint8_t index)
	{
		if(SYSERR(write(this->fd_i2cbus, &index, 1)) != 1)
			throw "failed to set register address";

		uint8_t value;
		if(SYSERR(read(this->fd_i2cbus, &value, 1)) != 1)
			throw "read register failed (8bit)";

		return value;
	}

	void TBME280::ReadRegister(const uint8_t index, const uint8_t n_bytes, void* const data)
	{
		if(SYSERR(write(this->fd_i2cbus, &index, 1)) != 1)
			throw "failed to set register address";

		if(SYSERR(read(this->fd_i2cbus, data, n_bytes)) != n_bytes)
			throw "read register failed (batch)";
	}

	template<typename T>
	static void Swap(T& a, T& b)
	{
		const T tmp = a;
		a = b;
		b = tmp;
	}

	void TBME280::Refresh()
	{
		union
		{
			struct
			{
				uint8_t press_msb;
				uint8_t press_lsb;
				uint8_t press_reserved : 4,
						press_xlsb : 4;

				uint8_t temp_msb;
				uint8_t temp_lsb;
				uint8_t temp_reserved : 4,
						temp_xlsb : 4;

				uint8_t hum_msb;
				uint8_t hum_lsb;
			};

			uint8_t octets[8];
		} regfile;

		if(sizeof(regfile) != 8)
			throw "logic error (regfile not 64bit)";

		ReadRegister(0xF7, 8, &regfile.octets);

		if(regfile.temp_reserved != 0)
			throw "logic error (temp_reserved != 0)";

		if(regfile.press_reserved != 0)
			throw "logic error (press_reserved != 0)";

		const uint32_t adc_T = (uint32_t)regfile.temp_msb  << 12 | (uint32_t)regfile.temp_lsb  << 4 | (uint32_t)regfile.temp_xlsb;
		const uint32_t adc_P = (uint32_t)regfile.press_msb << 12 | (uint32_t)regfile.press_lsb << 4 | (uint32_t)regfile.press_xlsb;
		const uint16_t adc_H = (uint16_t)regfile.hum_msb << 8 | (uint16_t)regfile.hum_lsb;

		if(DEBUG) fprintf(stderr, "[DEBUG] adc_T  =  % 6u\n", adc_T);
		if(DEBUG) fprintf(stderr, "[DEBUG] adc_P  =  % 6u\n", adc_P);
		if(DEBUG) fprintf(stderr, "[DEBUG] adc_H  =  % 6hu\n", adc_H);

		int32_t t_fine = 0;
		this->temperature = bosch::BME280_compensate_T_int32(adc_T, t_fine, this->calibration) / 100.0;
		if(DEBUG) fprintf(stderr, "[DEBUG] t_fine = % 6d\n", t_fine);

		const uint32_t press_q24_8 = bosch::BME280_compensate_P_int64(adc_P, t_fine, this->calibration);
		this->pressure = (double)(press_q24_8 >> 8) + ((press_q24_8 & 0xff) / 256.0);

		const uint32_t hum_q22_10 = bosch::bme280_compensate_H_int32(adc_H, t_fine, this->calibration);
		this->humidity = (double)(hum_q22_10 >> 10) + ((hum_q22_10 & 0b1111111111) / 1024.0);

		if(DEBUG) fprintf(stderr, "[DEBUG] temperature = %lf\n", this->temperature);
 		if(DEBUG) fprintf(stderr, "[DEBUG] pressure    = %lf\n", this->pressure);
 		if(DEBUG) fprintf(stderr, "[DEBUG] humidity    = %lf\n", this->humidity);
	}

	void TBME280::Reset()
	{
		SYSERR(ioctl(this->fd_i2cbus, I2C_SLAVE, this->address));

		const uint8_t chip_id = ReadRegister(0xD0);
		fprintf(stderr, "[INFO] Chip ID = %02hhx\n", chip_id);

		if(chip_id != 0x60)
			throw "this is not a BME280 (expected Chip ID = 0x60) => not supported by this driver";

		// execute chip reset
		WriteRegister(0xE0, 0xB6);
		usleep(100 * 1000);

		// put device to sleep
		WriteRegister(0xF4, 0x00);

		// 20ms sleep (when in normal mode), x2 filter, !spi mode => i2c mode
		WriteRegister(0xF5, 0b11100100);

		// read calibration data
		ReadRegister(0x88, sizeof(this->calibration.part1), this->calibration.part1);
		ReadRegister(0xE1, sizeof(this->calibration.part2), this->calibration.part2);
		this->calibration.ConvertEndianity();

		// undefined (00000), x16 humidity oversampling (101)
		WriteRegister(0xF2, 0b00000101);

		// x16 temperature oversampling (101), x16 pressure oversampling (101), normal mode (11)
		WriteRegister(0xF4, 0b10110111);

		// the first readout always seems to be garbage
		this->Refresh();

		// give the chip some time to complete at least one measurment cycle
		usleep(50 * 1000);
		this->Refresh();
	}

	TBME280::TBME280(const char* const i2c_bus_device, const uint8_t address) : fd_i2cbus(SYSERR(open(i2c_bus_device, O_RDWR | O_CLOEXEC | O_SYNC))), address(address), temperature(-100000.0), humidity(-100000.0), pressure(-100000.0)
	{
		Reset();
	}

	TBME280::TBME280(const int fd_i2cbus, const uint8_t address) : fd_i2cbus(fd_i2cbus), address(address), temperature(-100000.0), humidity(-100000.0), pressure(-100000.0)
	{
		Reset();
	}

	TBME280::~TBME280()
	{
		close(this->fd_i2cbus);
	}
}
