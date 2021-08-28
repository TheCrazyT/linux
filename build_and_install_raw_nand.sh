#!/bin/bash


export CURPWD=$PWD
cd ./drivers/mtd/nand
make -C $CURPWD M=$PWD clean
make KBUILD_EXTRA_SYMBOLS='$CURPWD/lib/Module.symvers' -C $CURPWD M=$PWD modules
sudo make -C $CURPWD M=$PWD modules_install
sudo cp $CURPWD/lib/bch.ko /usr/lib/modules/5.10.17-v7+/kernel/lib/
sudo depmod -a
