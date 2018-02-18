FROM ubuntu

RUN apt-get -y update
RUN apt-get -y install python3-pip

RUN pip3 install pyserial pyusb

RUN apt-get -y install gcc-arm-none-eabi gdb-arm-none-eabi openocd git
RUN apt-get -y install vim

VOLUME /odrive
VOLUME /dev/bus/usb

CMD cd /odrive/Firmware && /bin/bash
