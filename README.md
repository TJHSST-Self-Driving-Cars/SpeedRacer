# SpeedRacer

![story_routine](https://user-images.githubusercontent.com/37602685/193711544-4a9c5755-c374-404a-82b1-0a0c73c81a66.jpg)


# About
SpeedRacer is a 1/18 scale autonomous R/C car which incorporates an RPLidar (spinning lidar) and other components for autonomy, built by TJHSST's Self-Driving Cars Team. This GitHub repository contains code, schematics and CAD files for the microcontroller version of the car - which uses an ESP32 as the main computational device. 

* V0 - Nvidia Jetson, RPLidar
* V1 - ESP32, RPLidar
* V2(current) - ESP32, RPLidar (spinning LIDAR), TFMini (point distance sensor)


# Setup

To set up arduino for the ESP32, follow [these instructions](https://microcontrollerslab.com/install-esp32-arduino-ide/) or any other similar resource. This will allow you to select the correct board, which is called "DOIT ESP32 DEVKIT V1".

In addition, the ServoESP32 library should also be installed via the library manager.

The code should compile after this.
