Build
=====

```sh
cp config.example .config
make
```

Build time configuration
========================

Several build time options are available.
See [config.example](config.example) for a full listing.
The provided config.example is a generally useful default.

Debugging
=========

To enable debug printing for kernels build with CONFIG_DYNAMIC_DEBUG,
after module is loaded.

echo "module amc_pico +p" > /sys/kernel/debug/dynamic_debug/control

See https://www.kernel.org/doc/Documentation/dynamic-debug-howto.txt

ABI
===

One chardev is created for each pico8 card.
By default this is named with the PCI identifier as
for example ```/dev/amc_pico_0000:03:00.0```.

read()
------

This device may be read(), which arms the card
for acquisition, and returns when the requested number
of samples have been received.

The read() buffer size should be a multiple 32 bytes,
which is one 4 byte sample from each of eight channels.

A read will block until the acquisition logic is triggered,
or until ```ioctl(..., ABORT_READ)``` is issued (see ioctl section).

Returns the number of bytes read, or sets ```errno==ECANCELED```.

```errno==ERESTARTSYS``` may also be encountered if the syscall is
interrupted for other reasons.

Only one concurrent read() is allowed on each device.

ioctl()
-------

The header [amc_pico.h](amc_pico.h) defines several
macros for use with ioctl().

```
uint32_t ver = 0;
ioctl(fd, GET_VERSION, &ver);
if(ver!=GET_VERSION_CURRENT)
{ /* oops, kernel module version is different.  Can't proceed */ }
```

The GET_VERSION ioctl() should be the first one issued.
The result should be compared with GET_VERSION_CURRENT,
and/or other versions supported,
to determine if a compatible kernel module is loaded.

```
ioctl(fd, ABORT_READ);
```

Cause read() to return w/ errno==ECANCELED.
If a read() is in progress it returns immediately.
If no read() is in progress, then the next read() will return immediately.

After one read() has returned with errno==ECANCELED, subsequent read() calls
will block as normal.


```
SET_RANGE
GET_RANGE
SET_FSAMP
GET_FSAMP
SET_TRG
SET_RING_BUF
SET_GATE_MUX
SET_CONV_MUX
```

```
uint32_t site_id, site_ver;
ioctl(fd, GET_SITE_ID, &site_id);
ioctl(fd, GET_SITE_VERSION, &site_ver);
switch(site_id) {
case USER_SITE_NONE: /* stock FW from caenels */ break;
case USER_SITE_FRIB: /* FRIB customized FW */ break;
}
```

This provides a means for user applications
to detect and make use of custom firmware features.

ABI History
===========

Version 1 -> 2
--------------

* Define GET_VERSION_CURRENT as 2
* GET_VERSION which returns 2
* Update existing ioctl() except GET_VERSION to accurately reflect data direction.  Only ABI change.
* Add GET_SITE_ID, GET_SITE_VERSION, and SET_SITE_MODE ioctl()s
* Define USER_SITE_NONE and USER_SITE_FRIB

Version 0 -> 1
--------------

* On success, read() now returns the number of bytes stored instead of 0.  This removes the need for the GET_B_TRANS ioctl().
* To accomodate externel clock sources, GET_FSAMP/SET_FSAMP operate on sample clock divider instead of absolute frequency.
* Add GET_VERSION ioctl() which returns 1
* Add ABORT_READ ioctl()
