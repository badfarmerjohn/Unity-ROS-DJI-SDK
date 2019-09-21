# Using the Manifold

## Warnings
* Do not upgrade linux distribution of the Manifold. Only Ubuntu 14.04 (trusty) is supported.
* Do not uninstall (apt-get remove) python3. Almost all the built-in programs are dependent on it.


## Update the Manifold
In a terminal
```
sudo apt-get update
sudo apt-get upgrade
```
Search "software updater" in the launcher and run the software updater. *Do not upgrade linux distro, however*



## Installing ROS (Jade)
*ROS Jade is the latest distribution supported on Ubuntu 14.04*  
If you are using a Manifold that supports a later version of ROS, the instructions will need to be adapted for that version. It might even support installing from apt-get instead of building from source.
### Building ROS Jade from source
* http://wiki.ros.org/jade/Installation/Source  
*Cuz installing it using apt-get just doesn't work*  
Need to update add ROS repositories and update its GPG keys
* https://discourse.ros.org/t/new-gpg-keys-deployed-for-packages-ros-org/9454
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-key del 421C365BD9FF1F717815A3895523BAEEB01FA116

sudp apt-get update
```
In home directory:
```
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential

sudo rosdep init
rosdep update
```
Install the recommended desktop install
* The base installation is missing pretty much all packages needed by DJI ROS SDK and Rosbridge.
* The desktop-full installation takes a long time and doesn't work due to missing dependencies or something.

This is the part that takes the longest.
```
mkdir ~/ros_catkin_ws
cd ~/ros_catkin_ws

rosinstall_generator desktop --rosdistro jade --deps --wet-only --tar > jade-desktop-wet.rosinstall
wstool init -j4 src jade-desktop-wet.rosinstall
rosdep install --from-paths src --ignore-src --rosdistro jade -y

./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release
```

### Auto-sourcing Terminal
Add `source ~/ros_catkin_ws/install_isolated/setup.bash` to the end of the `~/.bashrc` file.  
*Note: this is not necessary if you're installing ROS through apt-get or some other method that adds ROS binaries to the PATH variable*


## Installing the DJI SDK
* http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started
```
mkdir ~/DJI
cd ~/DJI

git clone https://github.com/dji-sdk/Onboard-SDK
cd Onboard-SDK
mkdir build
cd ./build
cmake ..
make
sudo make install
```


## Installing DJI ROS Packages
* http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started
* https://developer.dji.com/guidance-sdk/documentation/application-development-guides/index.html#Linux
* https://dl.djicdn.com/downloads/manifold/en/Using_Manifold_to_Control_Matrice_100_en.pdf
```
mkdir ~/DJI/dji_catkin_ws
cd ~/DJI/dji_catkin_ws
touch dji.rosinstall
```
Add to the file `dji.rosinstall`
The first repository is needed since ROS Jade is somehow missing nmea message format which is needed by DJI's OSDK (for RTK GPS messages, I guess?).
```
- git: {local-name: nmea_msgs, uri: 'https://github.com/ros-drivers/nmea_msgs.git'}
- git: {local-name: osdk-ros, uri: 'https://github.com/dji-sdk/Onboard-SDK-ROS.git'}
- git: {local-name: guidance-sdk, uri: 'https://github.com/dji-sdk/Guidance-SDK-ROS.git'}
```
Install opencv if necessary
* Useful link: https://docs.opencv.org/3.4/d6/d15/tutorial_building_tegra_cuda.html
```
sudo apt-get install libopencv-dev
```
Proceed in the terminal:
```
wstool init -j4 src dji.rosinstall
wstool update -t src

rosdep install --from-paths src --ignore-src --rosdistro jade -y

catkin_make
```
Entering Configuration Info
```
source ~/DJI/dji_catkin_ws/devel/setup.bash
roscd dji_sdk
```
Edit `./launch/sdk.launch`, specifically the following fields:
* `serial_name`: If you are using the UART2 port on the Manifold, then set it to `/dev/ttyTHS1`
* `baud_rate`: set this to the same number configured for the drone through DJI Assistant 2.
  * Use 230400 for Matrice 100's.
  * Use 921600 for drones with the A3/N3 flight controller.
* `app_id`: the App ID from DJI's developer portal. You may need to create a new project in the developer portal to obtain one.
* `enc_key`: the Applicaiton Key from DJI's developer portal. You may need to create a new project in the developer portal to obtain one.


### Installing DJI Guidance and libusb binaries
Useful Links:
* https://github.com/dji-sdk/Guidance-SDK-ROS
* https://github.com/libusb/libusb
  * https://github.com/libusb/libusb/blob/master/README.git
* https://developer.dji.com/guidance-sdk/documentation/application-development-guides/index.html
* https://developer.dji.com/onboard-sdk/documentation/M210-Docs/oes-checklist.html

This is useful for using the Guidance system on the Matrice 100.
```
sudo cp ~/DJI/dji_catkin_ws/src/guidance-sdk/lib/x86/libDJI_guidance.so /usr/local/lib/
```
Install libusb-1.0.9
```
sudo apt-get install libudev-dev autoconf libtool
sudo apt-get install --reinstall ca-certificates
cd ~/Downloads
git clone https://github.com/libusb/libusb.git
cd ./libusb
./autogen.sh
./configure
make
sudo make install
```
Give usb rights to guidance ros node (so that it doesn't need to be run as root)
```
sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"fff0\", ATTR{idProduct}==\"d009\", MODE=\"0666\"" > /etc/udev/rules.d/51-guidance.rules'
```
To give usb rights to the DJI SDK to talk over usb-to-ttl (for non-matrice 100 models)
```
sudo sh -c 'echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"2ca3\", MODE=\"0666\"" > /etc/udev/rules.d/DJIDevice.rules'
sudo usermod -a -G dialout $USER
```
Restart
```
sudo reboot
```

### Running DJI SDK Node
```
source ~/DJI/dji_catkin_ws/devel/setup.bash
roslaunch dji_sdk sdk.launch
```

## Notes
Useful Links for Matrice 210:
* https://developer.dji.com/onboard-sdk/documentation/M210-Docs/oes-checklist.html
* https://developer.dji.com/onboard-sdk/documentation/M210-Docs/aircraft-checklist.html
* https://developer.dji.com/onboard-sdk/documentation/M210-Docs/simulation-checklist.html
* https://developer.dji.com/onboard-sdk/documentation/M210-Docs/real-world-test-checklist.html

* The first time you run this, you need to have the phone running DJI GO (for the Matrice 100) / DJI GO 4 (for the Matrice 210 series) connected to the paired controller in order to activate the drone. Upon launching the OSDK node, it will activate the drone and then error. Simply reboot the drone and restart the node.

* The USB switch for the Matrice 210 series determines which of three modes the usb port is used for, depending on the state of the switch:
  * Left: connect the flight controller to computer / DJI Assistant 2 for Matrice
    * DJI Assistant talks to / configures the flight controller.
    * This mode is also necessary for the onboard computer to get advanced sensing data (eg. camera/ultrasonic feeds).
  * Middle: connect the RTK module to the computer / DJI Assistant 2 for Matrice.
  * Right: connect the drone to the phone connected to the USB port.
    * Not really useful for anything other than updating the drone through a mobile device.

* The RTK module needs to be activated by a PC before the drone can be used. Do this by putting the USB switch to RTK mode and connect the drone to the computer running DJI Assistant 2 for Matrice. Remember to switch the USB mode switch back to the left mode.

* For the Matrice 210, check the configuration/designation of the expansion port pins to correctly connect the USB-to-TTY cable. The OSDK-Tx pin means it is the pin through which the **drone** transmits information, so the Rx pin of the USB-to-TTY cable should be connected there. Likewise, the OSDK-Rx pin is the pin through which the **drone** receives information, so the Tx pin of the USB-to-TTY cable should be connected there.
  * If things don't work, you can try swapping the Tx and Rx pins.

```
source ~/DJI/dji_catkin_ws/devel_isolated/setup.bash
roslaunch dji_sdk sdk.launch
```
If configured correctly, there should be a bunch of topics (check using the `rostopic list` command) starting with `/dji_sdk/`.
Some useful info on the topics can be found here: http://wiki.ros.org/dji_sdk. Useful services are:
* `/dji_sdk/sdk_control_authority`
  * Parameters
    * 1: request control authority
    * 0: release control authority
* `dji_sdk/drone_arm_control`
  * Parameters
    * 1: Arm the motors
    * 0: Disarm the motors
* `/dji_sdk/drone_task_control`
  * Parameters
    * 1: Go home
    * 4: Takeoff (currently not working)
    * 6: Landing
You can run them from a terminal (local or ssh) of the computer running the ROS Onboard SDK itself by doing `rosservice call <service_name> <parameters>`.


## Rosbridge

### Installing Rosbridge
If you're ROS version is compatible with installing ROSBridge through more convenient methods (such as apt-get), you can do that instead of using the steps, which pretty much build ROSBridge from the source.
```
mkdir ~/rosbridge_catkin_ws
cd ~/rosbridge_catkin_ws

touch rosbridge.rosinstall
```
Add to file `rosbridge.rosinstall`
```
- git: {local-name: rosauth, uri: 'https://github.com/GT-RAIL/rosauth.git'}
- git: {local-name: rosbridge, uri: 'https://github.com/RobotWebTools/rosbridge_suite.git', version: indigo}
```
Install Rosbridge
```
wstool init -j4 src rosbridge.rosinstall
wstool update -t src

rosdep install --from-paths src --ignore-src --rosdistro jade -y

catkin_make
```

### Running Rosbridge
```
source ~/rosbridge_catkin_ws/devel_isolated/setup.bash
roslaunch rosbridge_server rosbridge_websocket.launch
```


### Installing Image_View
image_view is useful for viewing topics that produce a camera image.
If you're ROS version is compatible with installing image_view through more convenient methods (such as apt-get), you can do that instead of using the steps, which pretty much build image_view from the source.
```
mkdir ~/imageview_catkin_ws
cd ~/imageview_catkin_ws

touch image_view.rosinstall
```
Add to file `image_view.rosinstall`
```
- git: {local-name: image-common, uri: 'https://github.com/ros-perception/image_common.git', version: 'hydro-devel'}
- git: {local-name: geometry2, uri: 'https://github.com/ros/geometry2.git', version: 'indigo-devel'}
- git: {local-name: image-pipeline, uri: 'https://github.com/ros-perception/image_pipeline.git', version: 'indigo'}
```
Install image_view
```
wstool init src image_view.rosinstall

rosdep install --from-paths src --ignore-src --rosdistro jade -y
catkin_make
```

## Installing Driver for TP-Link Archer T4U V1
Useful Links:
* https://askubuntu.com/questions/802205/how-to-install-tp-link-archer-t4u-driver
* https://github.com/abperiasamy/rtl8812AU_8821AU_linux

The Manifold does not come with built-in WiFi (at the time of writing). Though it certainly isn't the only option, this USB WiFi dongle is tested to work with the Manifold.
```
cd ~/Downloads
git clone https://github.com/abperiasamy/rtl8812AU_8821AU_linux.git
cd ./rtl8812AU_8821AU_linux
```
Edit the Makefile, making these changes:
* Comment out the line `EXTRA_CFLAGS += -Werror`
* Find the line with `ARCH ?= $(SUBARCH)` and change it to `ARCH ?= arm`
  * Otherwise it will try to compile with `ARCH ?= armv7l` which doesn't work.
You may need to change these instructions if using a different version of the Manifold (not v1).
Proceed with installation.
```
make
sudo make install
sudo apt-get install dkms
sudo modprobe -a rtl8812au
```


## Reflashing the Manifold
1. Download the OS image from DJI: https://dl.djicdn.com/downloads/manifold/manifold_image_v1.0.tar.gz
  * Do this on a linux machine that's not the manifold.
2. `sudo tar -xvpzf <path_to_downloaded_file>/manifold_image_v1.0.tar.gz`
  * Do this on the non-Manifold nonlinux machine
3. After the Manifold has booted, press the reset button while holding down the recovery button. Then release the recovery button.
4. Connect the recovery micro-USB port to the non-Manifold linux machine.
5. Check that the manifold is being registered by the non-Manifold linux machine by running lsusb and checking for a device with Nvidia in the name.
6. `cd <extracted manifold_image_folder>/manifold_images`
7. `sudo ./flash.sh jetson-tk1 mmcblk0p1`


### Other Links
* https://github.com/jayrambhia/Vision/blob/master/OpenCV/C%2B%2B/disparity.cpp
