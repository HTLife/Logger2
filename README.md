Logger2
=======

Tool for logging RGB-D data from the ASUS Xtion Pro Live with OpenNI2

Should build on Linux, MacOS and Windows. Haven't built on Windows yet though, so someone feel free try it out!


## Dependencies

### Installing Boost and Boost.Build
https://github.com/LORD-MicroStrain/MSCL/blob/master/BuildScripts/buildReadme_Linux.md

1. Download boost_1_64_0.tar.bz2
2. In the folder to put boost, run:
   `tar --bzip2 -xf /path/to/boost_1_64_0.tar.bz2`
3. `cd to boost directory`
4. `./bootstrap.sh --prefix=path/to/installation/prefix`
5. `./b2 install`


### MSCL

```bash
git clone https://github.com/LORD-MicroStrain/MSCL.git
# update the path below to match your boost lib path
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/jenkins/boost/boost_1.64.0_installed/lib

#build using bjam/boost.build
#Note: pass the following flags to bjam if desired:
#  --without-ssl           //removes OpenSSL dependency
#  --without-websockets    //removes Beast dependency
bjam MSCL//stage_c++ release --without-ssl --without-websocketsbjam MSCL//stage_c++ release
```

### Logger

```bash
sudo apt update
sudo apt install cmake 
sudo apt-get install qt4-dev-tools
sudo apt-get install libboost-all-dev
sudo apt-get install zlib1g-dev
sudo apt-get install libglm-dev
sudo apt-get install gtk2-engines-pixbuf gnome-themes-standard

sudo apt-get install libusb-1.0-0-dev
sudo apt-get install libudev-dev
git clone https://github.com/occipital/OpenNI2


https://sourceforge.net/projects/opencvlibrary/files/opencv-unix/2.4.9/opencv-2.4.9.zip/download
cmake -D BUILD_NEW_PYTHON_SUPPORT=OFF -D WITH_OPENCL=OFF -D WITH_OPENMP=ON -D INSTALL_C_EXAMPLES=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF -D WITH_QT=ON -D WITH_OPENGL=OFF -D WITH_VTK=OFF -D BUILD_PERF_TESTS=OFF -D BUILD_TESTS=OFF -D WITH_CUDA=OFF -D BUILD_opencv_gpu=OFF ..
```



Requires CMake, Boost, Qt4, OpenNI2, ZLIB and OpenCV. 

Grabs RGB and depth frames which can then be compressed (lossless ZLIB on depth and JPEG on RGB) or uncompressed. 

Frames can be cached in memory and written out when logging is finished, or streamed to disk. 

Multiple threads are used for the frame grabbing, compression and GUI. A circular buffer is used to help mitigate synchronisation issues that may occur. 

Supports disabling auto settings on the camera. 

The binary format is specified in Logger2.h

Run with -t to enable TCP streaming support on port 5698

If you get an error about not being able to connect to the device follow these instructions (credit to John McCormac)

1. Find the idVendor and idProduct by running lsusb (with example output):
```
  $ lsusb
```  
> Bus 003 Device 006: ID \<idVendorHere\>:\<idProductHere\> ASUS

2. Create a new usb rules file:
```
  $ sudo editor /etc/udev/rules.d/asus.rules
```
3. Paste in the below line with appropriate modifications (i.e. filling in the appropriate idVendor and idProduct listed above - 1d27 and 0601 for example):
```
  SUBSYSTEM=="usb", ATTR{idVendor}=="1d27", ATTR{idProduct}=="0601", GROUP="plugdev"
```
4. Double check you are indeed in the plugdev group, or change the group above accordingly
```
  $ groups
```
5. Reboot to take effect

<p align="center">
  <img src="http://mp3guy.github.io/img/Logger2.png" alt="Logger2"/>
</p>


