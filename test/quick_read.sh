#! /bin/sh

head -c $1 /dev/amc_pico | hexdump -e '8/4 "%+.1e " "\n"' | less
