# rpi-quadcopter
A quadcopter running a Raspberry Pi 4

This project is aimed at getting a quadcopter to run using a single [Raspberry Pi 4 8GB board](https://www.raspberrypi.org/products/raspberry-pi-4-model-b/) with a [PiCam](https://thepihut.com/collections/raspberry-pi-camera/products/raspberry-pi-camera-module), an [Adafruit BNO055 9DOF IMU](https://thepihut.com/products/adafruit-9-dof-absolute-orientation-imu-fusion-breakout-bno055) and an [Adafruit DPS310 Temp/altimeter](https://thepihut.com/products/adafruit-dps310-precision-barometric-pressure-altitude-sensor). It will run on [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html) on [Ubuntu 20.04 Server 64bit](https://ubuntu.com/download/raspberry-pi).  Initially I'll be focusing on software, reading the sensors, then sensor fusion and visual-inertial navigation using the PiCam.  I hope to be able to do everything up to PID control on the Pi but it may be necessary to also use an Arduino to control the motors (if I can't get an [ESC](https://dronenodes.com/drone-esc-electronic-speed-controller/) to do the job).

# Setup

Download the [Raspberry Pi Imager](https://www.raspberrypi.org/software/) onto your host machine.  When choosing the OS, select 'Other general purpose OS', then 'Ubuntu', then 'Ubuntu Server 20.04 64 bit'.

## WiFi
Unless you have a wired ethernet connection, you will want to connect the Pi to WiFi. We do this as follows
```
$ls /sys/class/net

eth0 lo wlan0
```

edit the network configuration file to add the section on wifis, using your wifi name (e.g. `wlan0`). Be careful with the indentation!

```
sudo nano /etc/netplan/50-cloud-init.yaml
```
```
# This file is generated from information provided by the datasource.  Changes
# to it will not persist across an instance reboot.  To disable cloud-init's
# network configuration capabilities, write a file
# /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg with the following:
# network: {config: disabled}
network:
    ethernets:
        eth0:
            dhcp4: true
            optional: true
    version: 2
    wifis:
        wlan0:
            optional: true
            access-points:
                "YourSSID":
                    password: "YourPassword"
            dhcp4: true
```

edit the cloud-init config to disable init
```
sudo nano /etc/cloud/cloud.cfg.d/99-disable-network-config.cfg
```
```
network: {config: disabled}
```

## Desktop Environment

Ubuntu 20.04 does not have a desktop environment, which makes it difficult (but not impossible) to develop on the Pi. Since we want to run ROS2 and make use of tools like `rqt_graph` and `rqt_plot` we want to install ubuntu desktop.  We do this as follows

```
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ubuntu-desktop
$ reboot
```

## ROS2

Install ROS2 following the [installation instructions](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

## PiCam

Now that ROS2 is installed, we can visualise the camera data with `cam2image` from `image_tools` as follows `ros2 run image_tools cam2image`.

![Screenshot from 2021-05-21 07-54-55](https://user-images.githubusercontent.com/937444/119106366-cf49e600-ba0d-11eb-8d12-4995750eeb12.png)


`cam2image` is quite limited as does not allow you to change the resolution or other settings.  As described in [this post](https://medium.com/swlh/raspberry-pi-ros-2-camera-eef8f8b94304) you can use the Linux Video4Linux driver instead.

```
$ mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
$ git clone --branch foxy https://gitlab.com/boldhearts/ros2_v4l2_camera.git
$ cd ..
$ colcon build
$ . install/local_setup.bash
```
Now we can run `ros2 run v4l2_camera v4l2_camera_node` and in another terminal `ros2 run rqt_image_view rqt_image_view`.

![Screenshot from 2021-05-21 08-23-44](https://user-images.githubusercontent.com/937444/119106457-e7216a00-ba0d-11eb-95e0-29f8565e2d09.png)


The ros2_v4l2_camera driver allows a lot of control over the camera, check it out.

## GPIO

GPIO should be easy on the RPi and an initial search for GPIO on Raspberry Pi on Ubuntu finds [this](https://ubuntu.com/tutorials/gpio-on-raspberry-pi#1-overview). Unfortunately this tutorial assumes 21.04 but we've installed 20.04 (for ROS2 LTS support).

Reading through [many](https://www.raspberrypi.org/forums/viewtopic.php?t=289084) [answers](https://raspberrypi.stackexchange.com/questions/40105/access-gpio-pins-without-root-no-access-to-dev-mem-try-running-as-root/40106) we find that we need to create a group that is permissioned to access /dev/gpiomem. This group already existing on Raspbian but not on Ubuntu 20.04. /dev/gpiomem is the part of memory where the GPIO bits map to the actual hardware. We do this as follows by creating a new `gpio` group, adding the current user to that group, giving the group ownership rights to /dev/gpiomem and then allowing group read/write access to /dev/gpiomem.  A reboot is required for the new group and permissions to take effect.

```
$ sudo groupadd gpio
$ sudo usermod -a -G gpio $USER
$ sudo chown root:gpio /dev/gpiomem
$ sudo chmod g+rw /dev/gpiomem
$ reboot
``` 

we [can test](https://www.raspberrypi.org/forums/viewtopic.php?t=190662#p1197592) that we have access to the without requiring sudo rights by running 
```
$ echo "18" > /sys/class/gpio/export 
$ echo "out" > /sys/class/gpio/gpio18/direction
$ echo "1" > /sys/class/gpio/gpio18/value
```

Now we will install RPi.GPIO, an easy to use Python library for controlling the GPIO pins.
```
$ sudo pip3 install RPi.GPIO
```
And check that it works
```
$ python3 -c 'import RPi.GPIO as GPIO; print(dir(GPIO))'
['BCM', 'BOARD', 'BOTH', 'FALLING', 'HARD_PWM', 'HIGH', 'I2C', 'IN', 'LOW', 'OUT', 'PUD_DOWN', 'PUD_OFF', 'PUD_UP', 'PWM', 'RISING', 'RPI_INFO', 'RPI_REVISION', 'SERIAL', 'SPI', 'UNKNOWN', 'VERSION', '__builtins__', '__cached__', '__doc__', '__file__', '__loader__', '__name__', '__package__', '__path__', '__spec__', 'add_event_callback', 'add_event_detect', 'cleanup', 'event_detected', 'getmode', 'gpio_function', 'input', 'output', 'remove_event_detect', 'setmode', 'setup', 'setwarnings', 'wait_for_edge']

$ $ python3 -c 'import RPi.GPIO as GPIO; print(GPIO.VERSION)'
0.7.0
```
With this working we will be able to create a ROS2 python node to read and write to the GPIO pins.

