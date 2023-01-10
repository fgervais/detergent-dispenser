#!/usr/bin/env bash

for i in boot.py webrepl_cfg.py main.py secret.py
do
	echo -n "$i "
	ampy --port /dev/ttyUSB3 put $i
	echo "OK"
done

