#!/bin/bash


export CURPWD=$PWD
cd ./drivers/mtd/nand
make -C $CURPWD M=$PWD clean
make KBUILD_EXTRA_SYMBOLS='$CURPWD/lib/Module.symvers' -C $CURPWD M=$PWD modules
sudo make -C $CURPWD M=$PWD modules_install
sudo depmod -a
