#!/bin/bash

sudo dtoverlay -r smi-nand
sudo modprobe -r bcm2835_smi_nand
sudo modprobe -r bcm2835_smi
