#!/bin/sh

GPIO_DFU=21
GPIO_NRST=22
GPIO_RST=34

RESET_TIME=1
USB_WAIT_TIME=2

DFU_FILE_PATH=/tmp/lcm/alien-lcm-fw.dfu

[ ! -e /sys/class/gpio/gpio${GPIO_DFU}/value ] && \
  echo $GPIO_DFU > /sys/class/gpio/export
[ ! -e /sys/class/gpio/gpio${GPIO_NRST}/value ] && \
  echo $GPIO_NRST > /sys/class/gpio/export
[ ! -e /sys/class/gpio/gpio${GPIO_RST}/value ] && \
  echo $GPIO_RST > /sys/class/gpio/export

echo out > /sys/class/gpio/gpio${GPIO_DFU}/direction
echo out > /sys/class/gpio/gpio${GPIO_NRST}/direction
echo out > /sys/class/gpio/gpio${GPIO_RST}/direction

# Enter DFU mode
echo 1 > /sys/class/gpio/gpio${GPIO_DFU}/value
echo 0 > /sys/class/gpio/gpio${GPIO_NRST}/value
echo 1 > /sys/class/gpio/gpio${GPIO_RST}/value
sleep $RESET_TIME
echo 1 > /sys/class/gpio/gpio${GPIO_NRST}/value
echo 0 > /sys/class/gpio/gpio${GPIO_RST}/value
sleep $USB_WAIT_TIME

# Check DFU mode active
dfu-util -l | grep -q Flash || \
    ( echo "LCM could not enter DFU mode" > /dev/kmsg && exit 1 )

# Flash
dfu-util -a0 -D $DFU_FILE_PATH || \
    ( echo "LCM DFU flashing failed" > /dev/kmsg && exit 2 )

# Reset to normal mode
echo 0 > /sys/class/gpio/gpio${GPIO_NRST}/value
echo 1 > /sys/class/gpio/gpio${GPIO_RST}/value
echo 0 > /sys/class/gpio/gpio${GPIO_DFU}/value
sleep $RESET_TIME
echo 1 > /sys/class/gpio/gpio${GPIO_NRST}/value
echo 0 > /sys/class/gpio/gpio${GPIO_RST}/value
sleep $USB_WAIT_TIME

# Check normal mode OK
[ -e /dev/ttyACM* ] || \
    ( echo "No LCM TTY after flashing" > /dev/kmsg && exit 3 )
