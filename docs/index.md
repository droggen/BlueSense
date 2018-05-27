---
layout: default
title: test BlueSense
---


### Bluesense: an extensible wearable motion sensing and IoT platform
BlueSense is a 30x30mm extensible wearable/IoT platform designed to be functional out-of-the-box yet extensible. 
It's primary purpose is to be an inertial measurement unit for wearable applications based on the Invensense MPU9250 and an AT1284p microcontroller. 
Yet it is extensible for IoT applications with a number of features making it appealing for this purpose.

## BlueSense: a wearable motion sensing platform
As a wearable platforn, BlueSense is a tiny device which can be used to capture body movement and orientation with a 9DoF motion sensor. The data can either be logged on an SD card or streamed over USB or Bluetooth 2. BlueSense could be used to prototype a custom fitness tracker. Its extension ports allow to plug-in additional sensor modalities for research purposes. A display is being developed converting it into a smartwatch. 

## BlueSense: an IoT platform
As an IoT device it's main appeal is a true hardware off which allows to put the device entirely to sleep (everything is powered down, including voltage regulators), yet wake up at programmed intervals thanks to a real-time clock wakeup. This allows to achieve very long battery life, as the device can wake up at desired times (once an hour, once a day, ...) to acquire and send sensor data. The extension ports allow to plug in custom sensor modalities which make this device highly versatile.

## BlueSense: firmware
The application firmware has been designed to be useful for a wide range of applications out of the box. It has been optimised for high-speed motion data logging and streaming (500Hz), and high-speed external ADC acquisition (1+KHz), informed by the need of wearable motion tracking and activity recognition applications. 

## BlueSense: other characteristics
Besides USB, Bluetooth 2 and SD card interface with a FAT32 compatible filesystem, BlueSense has a real-time clock, a coulomb counter to measure it's own battery level and power consumption, true hardware off, and connectors for extensions. This makes the platform also suitable for sensor research, IoT applications, or as an compact alternative to other microcontroller boards.



<p>Description of <em>BlueSense</em> <a href="hardware">Hardware specifications</a></p>

<p>Main: <a href="main">main</a></p>
