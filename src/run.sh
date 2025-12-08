#!/bin/sh

# run commands for BeagleBone

echo "7" > /proc/sys/kernel/printk # allow KERN_INFO to print on terminal
ifconfig eth0 192.168.7.2 netmask 255.255.255.0 up # establish network

# installing kernel module
mknod /dev/walker c 61 0
insmod /root/walk.ko

# launch userspace program
./kwalk
