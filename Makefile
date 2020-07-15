.PHONY: clean all

all: bme280-csv

bme280-csv: bme280-csv.cpp bme280.cpp bme280.hpp Makefile
	g++ -Wall -Wextra -flto -O3 -march=native -fdata-sections -ffunction-sections -Wl,--gc-sections bme280-csv.cpp bme280.cpp -o bme280-csv

clean:
	rm -vf bme280-csv
