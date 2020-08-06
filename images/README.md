
== Backup ==
$ sudo fdisk -l
$ sudo dd bs=4M if=/dev/sdb of=racecar.img

== Shrink ==
# Ref https://raspberrypi.stackexchange.com/questions/46450/reduce-ubuntu-mate-16-04-img-file-size

# Create loopback
$ sudo modprobe loop 
$ sudo losetup -f  

# This will return the path to a free loopback device. This is /dev/loop0 for me
$ sudo losetup /dev/loop0 racecar.img
$ sudo partprobe /dev/loop0
$ sudo gparted /dev/loop0

# In gparted, resize the partition (may require to keep 1-2 GB free to not have errors)

# Close gparted and the loopback
$ sudo losetup -d /dev/loop0 

# Truncate the image
$ fdisk -lu racecar.img
Disk racecar.img: 29.7 GiB, 31914983424 bytes, 62333952 sectors
Units: sectors of 1 * 512 = 512 bytes
Sector size (logical/physical): 512 bytes / 512 bytes
I/O size (minimum/optimal): 512 bytes / 512 bytes
Disklabel type: dos
Disk identifier: 0x93df4e89

Device       Boot  Start      End  Sectors  Size Id Type
racecar.img1        2048   409599   407552  199M  c W95 FAT32 (LBA)
racecar.img2      409600 25178111 24768512 11.8G 83 Linux

# Note the last value under "End" column
$ sudo truncate --size=$[(25178111+1)*512] racecar.img

== Restore ==
# Use Etcher: https://www.balena.io/etcher/
