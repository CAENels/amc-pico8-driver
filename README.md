== Build ==

```sh
cp config.example .config
make
```

== Debugging ==

To enable debug printing for kernels build with CONFIG_DYNAMIC_DEBUG,
after module is loaded.

echo "module amc_pico +p" > /sys/kernel/debug/dynamic_debug/control

See https://www.kernel.org/doc/Documentation/dynamic-debug-howto.txt
