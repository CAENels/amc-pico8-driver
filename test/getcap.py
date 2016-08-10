#!/usr/bin/env python

import fcntl, sys, ctypes

# this module is generated with gen_py
import picodefs as defs

with open(sys.argv[1], "r+b") as F:
    I = ctypes.c_uint32(0)
    fcntl.ioctl(F, defs.GET_VERSION, ctypes.pointer(I))
    print 'IOCTL version', I
