#!/bin/sh

MOUNTPOINT=/tmp/lcm

ubivolid=`ubinfo /dev/ubi0 -N lcm-test 2> /dev/null | awk '/Volume/{print $3}'`

if [ ! -z $ubivolid ]; then
    fs=ubifs
    device=/dev/ubi0_$ubivolid
else
    fs=squashfs
    mtdid=`cat /proc/mtd | awk '/\"lcm\"/{print $1}' | tr -d \\:mtd`
    device=/dev/mtdblock${mtdid}
fi

[ -e $device ] || \
    ( echo "No LCM firmware volume" > /dev/kmsg && exit 1 )

if [ "$1" == "mount" ]; then
    grep -q $device /proc/mounts && exit 0

    mkdir -p $MOUNTPOINT
    mount -t $fs $device $MOUNTPOINT || \
        ( echo "Could not mount LCM firmware volume" > /dev/kmsg && exit 2 )
elif [ "$1" == "umount" ]; then
    umount $MOUNTPOINT || \
        ( echo "Could not umount LCM firmware volume" > /dev/kmsg && exit 3 )
else
    exit 100
fi
