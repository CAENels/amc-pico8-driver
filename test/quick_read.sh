#! /bin/bash


display_usage() {
	echo "AMC-Pico-8 quick read utility"
	echo ""
	echo "A small utility to read analog signals from AMC-Pico-8."
	echo "Each \"line\" is composed of 8 samples (in single precision"
	echo "floating point format)"
	echo ""
	echo "Usage:"
	echo "    $0 NR_LINES"
	echo ""
}

if [ $# -ne 1 ]; then
	display_usage
	exit 1
fi

if [[ ( $1 == "--help") || ($1 == "-h") ]]; then
	display_usage
	exit 0
fi

nr_bytes_per_line=32
nr_bytes=$(echo "$1*$nr_bytes_per_line" | bc)

head -c $nr_bytes /dev/amc_pico | hexdump -e '8/4 "%+.2e " "\n"' | less
