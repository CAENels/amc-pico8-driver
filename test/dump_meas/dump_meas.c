
// Copyright (c) 2015 CAEN ELS d.o.o.

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


#define BYTES_PER_LINE	(32)
#define OUT_FORMAT		"%+0.6e"

char AMC_PICO_DEV[128];

////////////////////////////////////////////////////////////////////////////////
/// \brief prints usage information

void print_usage(const char* name){
	printf("AMC-Pico-8 measurement dump\n");
	printf("\n");
	printf("Arguments:\n");
	printf("    --devfile DEVFILE  Device file in /dev")
	printf("    --nrsamp NRSAMP  Number of samples to read\n");
	printf("    --out FILENAME   Output file name\n");
	printf("    --force          Overwrites output file\n");
	printf("\n");
	printf("Example:\n");
	printf("    %s --devfile /dev/amc_pico_0000:05:00.0 --nrsamp 100 --out meas1.csv", name);
	printf("\n");
}

////////////////////////////////////////////////////////////////////////////////
/// \brief reads from AMC-Pico-8

int read_from_pico(uint32_t nrsamp, void *buf) {

	int fd = open(AMC_PICO_DEV, O_RDONLY | O_SYNC);
	if (fd<0) {
		perror("open()");
		return -1;
	}

	ssize_t bytes_read = read(fd, buf, nrsamp*BYTES_PER_LINE);

	close(fd);
	return !(bytes_read == nrsamp*BYTES_PER_LINE);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief dumps buffer to CSV file

int dump_to_file(void* buf, const char* filename, uint32_t nrsamp, int force) {

	FILE* f = fopen(filename, force ? "w" : "wx");
	if (f == NULL) {
		perror("fopen()");
		return -1;
	}

	// Print header
	fprintf(f, "Index,");
	for (int ch=0; ch<8; ch++)
		fprintf(f, "Channel %d,", ch);
	fprintf(f, "\n");

	float* d_ptr = (float*) buf;
	for (uint32_t line=0; line<nrsamp; line++){
		fprintf(f, "%d,", line);
		for (int ch=0; ch<8; ch++)
			fprintf(f, OUT_FORMAT ",", *d_ptr++);
		fprintf(f, "\n");
	}

	fclose(f);

	return 0;
}


////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {

	uint32_t nrsamp = 0;
	char filename[256] = {0};
	int force = 0;

	static struct option long_options[] = {
		{"help",    no_argument,       NULL, 'h' },
		{"devfile", required_argument, NULL, 'd' },
		{"nrsamp",  required_argument, NULL, 'n' },
		{"out",     required_argument, NULL, 'o' },
		{"force",   no_argument, NULL, 'f' },
		{0, 0, 0, 0 }
	};

	while (1) {
		int c;
		c = getopt_long(argc, argv, "n:o:fh", long_options, NULL);
		if (c == -1)
			break;

		switch(c){
		case 'h':
			print_usage(argv[0]);
			return 0;
		case 'd':
			strncpy(AMC_PICO_DEV, optarg, sizeof(AMC_PICO_DEV));
		case 'n':
			sscanf(optarg, "%i", &nrsamp);
			break;
		case 'o':
			strncpy(filename, optarg, 255);
			break;
		case 'f':
			force = 1;
			break;
		}
	}

	if ((nrsamp != 0) && strlen(filename)) {
		void *buf = malloc(BYTES_PER_LINE*nrsamp);
		if (buf == NULL) {
			perror("malloc()");
			return -1;
		}

		int rc;
		rc = read_from_pico(nrsamp, buf);
		if (rc < 0){
			printf("Error reading from AMC-Pico-8\n");
			free(buf);
			return -1;
		}

		dump_to_file(buf, filename, nrsamp, force);

		free(buf);
	}

	return 0;
}
