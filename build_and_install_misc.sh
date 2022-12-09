#!/bin/bash

export CURPWD=$PWD

cd drivers/misc/
make -C $CURPWD M=$PWD clean
make -C $CURPWD M=$PWD
sudo make -C $CURPWD M=$PWD modules_install
