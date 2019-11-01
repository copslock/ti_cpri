#!/bin/sh

user=`whoami`
if [[ $user != "root" ]];
then
echo "Should run by root only ! (use : sudo)"
exit 1
fi

if [ $# -eq 0 ]
  then
    echo "No arguments supplied ! (e.g, usage : sudo sh create-sdcard-omapl13x.sh /dev/sdb "
exit 1
fi

DRIVE=$1

# Un mount all the partitions.
echo "Un mounting all the partitions..."

sudo umount $DRIVE"1"

dd if=/dev/zero of=$DRIVE bs=1024 count=1024

SIZE=`fdisk -l $DRIVE | grep Disk | awk '{print $5}'`

echo DISK SIZE - $SIZE bytes

CYLINDERS=`echo $SIZE/255/63/512 | bc`

# Start single partition starting at 2MB offset. First 2MB is used for boot image loading
sfdisk -D -H 255 -S 63 -C $CYLINDERS $DRIVE << EOF
2,,,*
EOF

mkfs.vfat -F 32 -n "boot" ${DRIVE}1
umount ${DRIVE}1

echo "Done !"
