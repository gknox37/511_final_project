#!/bin/bash
build/ARM/gem5.fast configs/example/se.py --caches --l2cache --debug-flags=Cache -c tests/test-progs/hello/bin/arm/linux/hello 
