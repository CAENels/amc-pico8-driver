#!/usr/bin/env python

import fcntl, sys, ctypes

# this module is generated with gen_py
import picodefs as defs

with open(sys.argv[1], "r+b") as F:
    I = ctypes.c_uint32(42)
    print fcntl.ioctl(F, defs.GET_VERSION, ctypes.pointer(I), True)
    print 'IOCTL version', I
    print fcntl.ioctl(F, defs.GET_SITE_ID, ctypes.pointer(I), True)
    print 'IOCTL site version', I
