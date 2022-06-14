#!/bin/sh
echo $PWD

if [ $(uname -m) = "x86_64" ];then
 echo 'OS : x86_64 bits'
 export LD_LIBRARY_PATH="$PWD/lib/linux_64bit:$LD_LIBRARY_PATH"
else
 echo "OS : 32bits"
 export LD_LIBRARY_PATH="$PWD/lib/linux_32bit:$LD_LIBRARY_PATH"
fi

echo $LD_LIBRARY_PATH
./USB_SPI_Test
