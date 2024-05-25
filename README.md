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

### mix usage

Last note is that all the options can be used at the same time, the dtoverlay will looks like this:
```
camera_auto_detect=0
dtoverlay=imx585,always-on,mono,always-on
```
