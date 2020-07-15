#include <sys/uio.h>
#include <unistd.h>
#include <stdio.h>
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "bme280.hpp"

#define SYSERR(expr) (([&](){ const auto r = ((expr)); if( (long)r == -1L ) { throw #expr; } else return r; })())

// // Returns temperature in DegC, resolution is 0.01 DegC. Output value of “5123” equals 51.23 DegC.
// // t_fine carries fine temperature as global value
//
//
// // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
// uint32_t BME280_compensate_P_int64(int32_t adc_P)
// {
// 	int64_t var1, var2, p;
// 	var1 = ((int64_t)t_fine) - 128000;
// 	var2 = var1 * var1 * (int64_t)dig_P6;
// 	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
// 	var2 = var2 + (((int64_t)dig_P4)<<35);
// 	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
// 	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
// 	if (var1 == 0)
// 	{
// 		return 0; // avoid exception caused by division by zero
// 	}
// 	p = 1048576-adc_P;
// 	p = (((p<<31)-var2)*3125)/var1;
// 	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
// 	var2 = (((int64_t)dig_P8) * p) >> 19;
// 	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
// 	return (uint32_t)p;
// }
//
// // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// // Output value of “47445” represents 47445/1024 = 46.333 %RH
// uint32_t bme280_compensate_H_int32(int32_t adc_H)
// {
// 	int32_t v_x1_u32r;
// 	v_x1_u32r = (t_fine - ((int32_t)76800));
// 	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) *
// 	v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) + 8192) >> 14));
// 	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dig_H1)) >> 4));
// 	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
// 	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
// 	return (uint32_t)(v_x1_u32r>>12);
// }


namespace bme280
{
	bool DEBUG = false;

	void TBME280::calibration_t::ConvertEndianity()
	{
		dig_T1 = ntohs(dig_T1);
		dig_T2 = ntohs(dig_T2);
		dig_T3 = ntohs(dig_T3);
		dig_P1 = ntohs(dig_P1);
		dig_P2 = ntohs(dig_P2);
		dig_P3 = ntohs(dig_P3);
		dig_P4 = ntohs(dig_P4);
		dig_P5 = ntohs(dig_P5);
		dig_P6 = ntohs(dig_P6);
		dig_P7 = ntohs(dig_P7);
		dig_P8 = ntohs(dig_P8);
		dig_P9 = ntohs(dig_P9);
		dig_H2 = ntohs(dig_H2);
		dig_H4 = ntohs(dig_H4);
		dig_H5 = ntohs(dig_H5);
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

	void TBME280::Refresh()
	{
		struct
		{
			int64_t
				P : 24,
				T : 16,
				H : 24;
		} regfile;

		if(sizeof(regfile) != 8)
			throw "logic error";

		ReadRegister(0xF7, 8, &regfile);

		if(DEBUG) fprintf(stderr, "[DEBUG] regfile = %016llx\n", *(uint64_t*)&regfile);

// 		const int32_t adc_T = ntohs((int16_t)regfile.T);

		int32_t t_fine;

		{
			const int32_t adc_T = regfile.T;
			int32_t var1, var2;
			var1 = ((((adc_T>>3) - ((int32_t)calibration.dig_T1<<1))) * ((int32_t)calibration.dig_T2)) >> 11;
			var2 = (((((adc_T>>4) - ((int32_t)calibration.dig_T1)) * ((adc_T>>4) - ((int32_t)calibration.dig_T1))) >> 12) * ((int32_t)calibration.dig_T3)) >> 14;
			t_fine = var1 + var2;
			this->temperature = (double)((t_fine * 5 + 128) >> 8) / 1000.0;
		}

		{
			const int32_t adc_P = regfile.P;
			int64_t var1, var2, p;
			var1 = ((int64_t)t_fine) - 128000;
			var2 = var1 * var1 * (int64_t)calibration.dig_P6;
			var2 = var2 + ((var1*(int64_t)calibration.dig_P5)<<17);
			var2 = var2 + (((int64_t)calibration.dig_P4)<<35);
			var1 = ((var1 * var1 * (int64_t)calibration.dig_P3)>>8) + ((var1 * (int64_t)calibration.dig_P2)<<12);
			var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calibration.dig_P1)>>33;

			if (var1 != 0)
			{
				p = 1048576-adc_P;
				p = (((p<<31)-var2)*3125)/var1;
				var1 = (((int64_t)calibration.dig_P9) * (p>>13) * (p>>13)) >> 25;
				var2 = (((int64_t)calibration.dig_P8) * p) >> 19;
				p = ((p + var1 + var2) >> 8) + (((int64_t)calibration.dig_P7)<<4);
			}
			else
				p = 0;

			this->pressure = (double)p;
		}

		{
			const int32_t adc_H = regfile.H;
			int32_t v_x1_u32r;
			v_x1_u32r = (t_fine - ((int32_t)76800));
			v_x1_u32r = (((((adc_H << 14) - (((int32_t)calibration.dig_H4) << 20) - (((int32_t)calibration.dig_H5) *
			v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calibration.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calibration.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calibration.dig_H2) + 8192) >> 14));
			v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calibration.dig_H1)) >> 4));
			v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
			v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
			this->humidity = (uint32_t)(v_x1_u32r>>12);
		}

		if(DEBUG) fprintf(stderr, "[DEBUG] temperature = %lf\n", this->temperature);
		if(DEBUG) fprintf(stderr, "[DEBUG] pressure    = %lf\n", this->pressure);
		if(DEBUG) fprintf(stderr, "[DEBUG] humidity    = %lf\n", this->humidity);
	}

	void TBME280::Reset()
	{
		SYSERR(ioctl(this->fd_i2cbus, I2C_SLAVE, this->address));

		const uint8_t chip_id = ReadRegister(0xD0);
		fprintf(stderr, "[INFO] Chip ID = %02hhx\n", chip_id);

		// execute chip reset
		WriteRegister(0xE0, 0xB6);
		usleep(10 * 1000);

		// put device to sleep
		WriteRegister(0xF4, 0x00);

		// 0.5ms sleep (when in normal mode), x16 filter, !spi mode => i2c mode
		WriteRegister(0xF5, 0b00010000);

		// read calibration data
		ReadRegister(0x88, sizeof(this->calibration.part1), this->calibration.part1);
		ReadRegister(0xE1, sizeof(this->calibration.part2), this->calibration.part2);
		this->calibration.ConvertEndianity();

		// x16 humidity oversampling
		WriteRegister(0xF2, 0xff);

		// switch to normal mode, x16 pressure oversampling,  x16 temperature oversampling
		WriteRegister(0xF4, 0xff);

		// give the chip some time to complete at least one measurment cycle
		usleep(100 * 1000);
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
