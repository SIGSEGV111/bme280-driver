#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/file.h>
#include "bme280.hpp"

#define SYSERR(expr) (([&](){ const auto r = ((expr)); if( (long)r == -1L ) { throw #expr; } else return r; })())

static volatile bool do_run = true;

static void OnSignal(int)
{
	do_run = false;
}

int main(int argc, char* argv[])
{
	signal(SIGINT,  &OnSignal);
	signal(SIGTERM, &OnSignal);
	signal(SIGHUP,  &OnSignal);
	signal(SIGQUIT, &OnSignal);

	close(STDIN_FILENO);

	try
	{
		if(argc < 3)
			throw "need exactly two arguments: <i2c bus device> <location>";

		using namespace bme280;
		::bme280::DEBUG = true;

		TBME280 bme280(argv[1], 0x76);

		while(do_run)
		{
			timeval ts;

			bme280.Refresh();
			SYSERR(gettimeofday(&ts, NULL));

			SYSERR(flock(STDOUT_FILENO, LOCK_EX));
// 			printf("%ld.%06ld;\"%s\";\"BME280\";\"LUX\";%f\n", ts.tv_sec, ts.tv_usec, argv[2], bme280.Lux());
// 			fflush(stdout);
			SYSERR(flock(STDOUT_FILENO, LOCK_UN));

			usleep(1 * 1000 * 1000);
		}

		fprintf(stderr,"\n[INFO] bye!\n");
		return 0;
	}
	catch(const char* const err)
	{
		fprintf(stderr, "[ERROR] %s\n", err);
		perror("[ERROR] kernel message");
		return 1;
	}
	return 2;
}
