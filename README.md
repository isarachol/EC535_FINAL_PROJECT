Half-wlkaing man is the project aimming to develop a bipedel robots haing an ability to follow the selected object with safe distance. The steup instruction can be found below:

**Create Local Network**
- First you need to assign IP address using Interface Configuration to Laptop using Beagle Bone's ethernet IP address
  - To Find IP address: using ifconfig in command terminal in Beagle Bone and find IP address in th0 section

For Laptop: 
- Using IP address number except the last digit to create private channel
- Find the name your ethernet port and replace enp5s0 with it
- Command: sudo ifconfig enp5s0 192.168.7.1 netmask 255.255.255.0 up (assign IP address to ethernet part and then determine the capcaity of how many devices can be assigned to use the private channel)

For Beagle Bone: 
- ifconfig eth0 192.168.7.2 netmask 255.255.255.0 up

**Upload Kernel Module and Userspace**

In Beagle Bone: 
- Using command: rz (activate picocom)
- Then click Ctrl A and Ctrl S (trigger the laptop to prepare to send file)
- Then use the file path

**Run Object Detection and Kernel program**

First: Install Kernel Module (Beagle Bone)
1. Create Charater device
    - Command: mknod /dev/walker c 61 0
2. Insert Kernel Module into the device node
    - Command: insmod /root/walk.ko
3. Run Userspace command
    - Command: ./kwalk

Second: Run Object Detection Code (Laptop)
1. Compile Code with C++ and openCV flag
    - Command: g++ Test5.cpp -o Test5 `pkg-config --cflags --libs opencv4`
2. Run Code:
    - Command: ./Test5
  
Result: 
- The expected output is the laptop send real-time data using ethernet port to Beagle Bone. Beagle Bone uses Userspace as middleman to receive and send data to kernel.
  Kernel will use the sent data to control servo motor and distance sensor
- The object can be changed to be 4 types. First, it need to select the video window and then click spacebar to change the combination of object and color. 
