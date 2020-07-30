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

	// ---------------------------------------

	// Returns temperature in DegC, double precision. Output value of “51.23” equals 51.23 DegC.
	// t_fine carries fine temperature as global value
	static double BME280_compensate_T_double(const int32_t adc_T, int32_t& t_fine, const bme280::TBME280::calibration_t& cal)
	{
		double var1, var2, T;
		var1 = (((double)adc_T)/16384.0 - ((double)cal.dig_T1)/1024.0) * ((double)cal.dig_T2);
		var2 = ((((double)adc_T)/131072.0 - ((double)cal.dig_T1)/8192.0) * (((double)adc_T)/131072.0 - ((double)cal.dig_T1)/8192.0)) * ((double)cal.dig_T3);
		t_fine = (int32_t)(var1 + var2);
		T = (var1 + var2) / 5120.0;
		return T;
	}
	// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
	static double BME280_compensate_P_double(const int32_t adc_P, const int32_t t_fine, const bme280::TBME280::calibration_t& cal)
	{
		double var1, var2, p;
		var1 = ((double)t_fine/2.0) - 64000.0;
		var2 = var1 * var1 * ((double)cal.dig_P6) / 32768.0;
		var2 = var2 + var1 * ((double)cal.dig_P5) * 2.0;
		var2 = (var2/4.0)+(((double)cal.dig_P4) * 65536.0);
		var1 = (((double)cal.dig_P3) * var1 * var1 / 524288.0 + ((double)cal.dig_P2) * var1) / 524288.0;
		var1 = (1.0 + var1 / 32768.0)*((double)cal.dig_P1);
		if (var1 == 0.0)
		{
			return 0; // avoid exception caused by division by zero
		}
		p = 1048576.0 - (double)adc_P;
		p = (p - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)cal.dig_P9) * p * p / 2147483648.0;
		var2 = p * ((double)cal.dig_P8) / 32768.0;
		p = p + (var1 + var2 + ((double)cal.dig_P7)) / 16.0;
		return p;
	}
	// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
	static double bme280_compensate_H_double(const int32_t adc_H, const int32_t t_fine, const bme280::TBME280::calibration_t& cal)
	{
		double var_H;
		var_H = (((double)t_fine) - 76800.0);
		var_H = (adc_H - (((double)cal.dig_H4) * 64.0 + ((double)cal.dig_H5) / 16384.0 * var_H)) * (((double)cal.dig_H2) / 65536.0 * (1.0 + ((double)cal.dig_H6) / 67108864.0 * var_H * (1.0 + ((double)cal.dig_H3) / 67108864.0 * var_H)));
		var_H = var_H * (1.0 - ((double)cal.dig_H1) * var_H / 524288.0);
		if (var_H > 100.0)
			var_H = 100.0;
		else if (var_H < 0.0)
			var_H = 0.0;
		return var_H;
	}
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
		if(DEBUG) fprintf(stderr, "[DEBUG] switching BME280 into forced mode ...\n");
		// x16 temperature oversampling (101), x16 pressure oversampling (101), forced mode (01)
 		WriteRegister(0xF4, 0b10110101);

		if(DEBUG) fprintf(stderr, "[DEBUG] waiting for BME280 to complete measurment ...\n");
		usleep(1000);
		while((ReadRegister(0xF3) & 0b00000011) != 0)
			usleep(1000);

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

		if(DEBUG) fprintf(stderr, "[DEBUG] reading measurment data...\n");
		ReadRegister(0xF7, 8, &regfile.octets);

		if(regfile.temp_reserved != 0)
			throw "logic error (temp_reserved != 0)";

		if(regfile.press_reserved != 0)
			throw "logic error (press_reserved != 0)";

		const uint32_t adc_T = (uint32_t)regfile.temp_msb  << 12 | (uint32_t)regfile.temp_lsb  << 4 | (uint32_t)regfile.temp_xlsb;
		const uint32_t adc_P = (uint32_t)regfile.press_msb << 12 | (uint32_t)regfile.press_lsb << 4 | (uint32_t)regfile.press_xlsb;
		const uint16_t adc_H = (uint16_t)regfile.hum_msb << 8 | (uint16_t)regfile.hum_lsb;

		if(DEBUG) fprintf(stderr, "[DEBUG] adc_T  =  %6u\n", adc_T);
		if(DEBUG) fprintf(stderr, "[DEBUG] adc_P  =  %6u\n", adc_P);
		if(DEBUG) fprintf(stderr, "[DEBUG] adc_H  =  %6hu\n", adc_H);

		int32_t t_fine = 0;
		this->temperature = bosch::BME280_compensate_T_double(adc_T, t_fine, this->calibration);
		this->pressure = bosch::BME280_compensate_P_double(adc_P, t_fine, this->calibration);
		this->humidity = bosch::bme280_compensate_H_double(adc_H, t_fine, this->calibration);

		if(DEBUG) fprintf(stderr, "[DEBUG] t_fine = %6d\n", t_fine);
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
		if(DEBUG) fprintf(stderr, "[DEBUG] sending chip-reset command ...\n");
		WriteRegister(0xE0, 0xB6);

		if(DEBUG) fprintf(stderr, "[DEBUG] waiting for BME280 to complete chip reset ...\n");
		usleep(10 * 1000);
		while((ReadRegister(0xF3) & 0b00000011) != 0)
			usleep(10 * 1000);

		// read calibration data
		if(DEBUG) fprintf(stderr, "[DEBUG] reading calibration data ...\n");
		ReadRegister(0x88, sizeof(this->calibration.part1), this->calibration.part1);
		ReadRegister(0xE1, sizeof(this->calibration.part2), this->calibration.part2);
		this->calibration.ConvertEndianity();

		if(DEBUG) fprintf(stderr, "[DEBUG] sending measurment config ...\n");

		// undefined (00000), x16 humidity oversampling (001)
 		WriteRegister(0xF2, 0b00000101);

		// x16 temperature oversampling (001), x16 pressure oversampling (001), sleep mode (00)
 		WriteRegister(0xF4, 0b10110100);

		// (000) 0.5ms sleep (when in normal mode), (000) filter off, (0) undefined, (0) !spi mode => i2c mode
		WriteRegister(0xF5, 0b00000000);

		if(DEBUG) fprintf(stderr, "[DEBUG] ready for measurment!\n");

		// the first measurment always seems to be garbage
		this->Refresh();
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
