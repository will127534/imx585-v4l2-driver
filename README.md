# Kernel Driver for IMX585

This guide provides detailed instructions on how to install the IMX585 kernel driver on a Linux system, specifically Raspbian.

## Special Thanks

Special thanks to Octopuscinema and Soho-enterprise, the driver is based on their imx585 driver here:
https://github.com/octopuscinema/linux-camera-support/blob/rpi-6.1.y/drivers/media/i2c/imx585.c

Special thanks to Sasha Shturma's Raspberry Pi CM4 Сarrier with Hi-Res MIPI Display project, the DKMS install script is adapted from the github project page: https://github.com/renetec-io/cm4-panel-jdi-lt070me05000


## Prerequisites

Before you begin the installation process, please ensure the following prerequisites are met:

- **Kernel version**: You should be running on a Linux kernel version 6.1 or newer. You can verify your kernel version by executing `uname -r` in your terminal.

- **Development tools**: Essential tools such as `gcc`, `dkms`, and `linux-headers` are required for compiling a kernel module. If not already installed, these can be installed using the package manager with the following command:
  
   ```bash 
   sudo apt install linux-headers dkms git
   ```
   
## Installation Steps

### Setting Up the Tools

First, install the necessary tools (`linux-headers`, `dkms`, and `git`) if you haven't done so:

```bash 
sudo apt install linux-headers dkms git
```

### Fetching the Source Code

Clone the repository to your local machine and navigate to the cloned directory:

```bash
git clone https://github.com/will127534/imx585-v4l2-driver.git
cd imx585-v4l2-driver/
```

### Compiling and Installing the Kernel Driver

To compile and install the kernel driver, execute the provided installation script:

```bash 
./setup.sh
```

### Updating the Boot Configuration

Edit the boot configuration file using the following command:

```bash
sudo nano /boot/config.txt
```

In the opened editor, locate the line containing `camera_auto_detect` and change its value to `0`. Then, add the line `dtoverlay=imx585`. So, it will look like this:

```
camera_auto_detect=0
dtoverlay=imx585
```

After making these changes, save the file and exit the editor.

Remember to reboot your system for the changes to take effect.

## dtoverlay options

### cam0

If the camera is attached to cam0 port, append the dtoverlay with `,cam0` like this:  
```
camera_auto_detect=0
dtoverlay=imx585,cam0
```

### always-on

If you want to keep the camera power always on (Useful for debugging HW issues, specifically this will set CAM_GPIO to high constantly), append the dtoverlay with `,always-on` like this:  
```
camera_auto_detect=0
dtoverlay=imx585,always-on
```

### mono

If you are using a monochrome varient, append the dtoverlay with `,mono` like this:  
```
camera_auto_detect=0
dtoverlay=imx585,mono
```

### Lane Count

If you want to use 2-lane for IMX585, append the dtoverlay with `,2lane` like this:  
```
camera_auto_detect=0
dtoverlay=imx585,2lane
```


### link-frequency

If you want to change the default link frequency of 1440Mbps/lane (720Mhz), you can chage it like the following:
```
camera_auto_detect=0
dtoverlay=imx585,link-frequency=297000000
```
Here is a list of available frequencies:
| Valid Frequency Value | Mbps/Lane | Max Framerate with 4K + 4 lane |
| -------- | -------- | -------- |
| 297000000|594 Mbps/Lane| 20.83|
| 360000000|720 Mbps/Lane| 25|
| 445500000|891 Mbps/Lane| 30|
| 594000000|1188 Mbps/Lane| 41.67|
| 720000000|1440 Mbps/Lane| 50|
| 891000000|1782 Mbps/Lane| 60|
| 1039500000|2079 Mbps/Lane| 75|

Notes that by default RPI5/RP1 has a limit of 400Mpix/s processing speed, without overclocking you will be limited to ~45 FPS @ 4K


### mix usage

Last note is that all the options can be used at the same time, the dtoverlay will looks like this:
```
camera_auto_detect=0
dtoverlay=imx585,always-on,mono,cam0,link-frequency=297000000
```
Imaging how many config I need to test.

