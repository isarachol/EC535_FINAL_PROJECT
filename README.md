# Autonomous Walking Robot (Half-body walking man) 
Half-body walking man is a project aiming to develop a simple bipedal robot capable of following a selected object at a safe distance. In this project, we created the mechanical design, electrical wiring, and three programs.

## Repo Structure
"main" branch
- CAD: directory containing STL files used to create this robot
- src: directory containing all final source codes and their Makefiles
  - laptop: contains object-detection code in cpp, which is meant to be implemented on a laptop
  - ul: contains userspace code which receives commands from the laptop code
  - km: contains kernel module code, which receive commands from userspace program and implements distance sensor and servo motors controls.

"burnt" branch is for development

## How to replicate our creation

**Components**
- Beagle Bone black (with lab5 kernel image, USB to TTL Serial Cable, and power supply) x 1
- HC-SR04 distance sensor x 1
- Logitech Orbit AF Webcam x 1
- Servo motors x 3
- LAN cable x 1
- Laptop with Linux OS x 1

**Wiring Diagram**
![alt text](https://github.com/isarachol/EC535_FINAL_PROJECT/blob/main/wiring_diagram.png "Wiring Diagram")

**Create Local Network**
- First you need to assign IP address using Interface Configuration to Laptop using Beagle Bone's ethernet IP address
  - To Find IP address: using ifconfig in command terminal in Beagle Bone and find IP address in th0 section

For Laptop: 
- Using IP address number except the last digit to create private channel
- Find the name your ethernet port using the command ` ls /sys/class/net/ ` and replace enp5s0 with it. There may be multiple names, look for names like eth0, enp2s0, or wlp3s0 (for wireless)
- Command: `sudo ifconfig enp5s0 192.168.7.1 netmask 255.255.255.0 up` (assign IP address to ethernet part and then determine the capcaity of how many devices can be assigned to use the private channel)

For Beagle Bone: 
- `ifconfig eth0 192.168.7.2 netmask 255.255.255.0 up`

**Make and Upload Kernel Module and Userspace**

Clone the src folder and make each file. You should have `walk.ko`, `walk`, and `Laptop`

In Beagle Bone: 
- Using command: rz (activate picocom)
- Then click Ctrl A and Ctrl S (trigger the laptop to prepare to send the file)
- Then use the file path

**Run Object Detection and Kernel program**

First: Install Kernel Module (Beagle Bone)
1. Create Charater device
    - Command: `mknod /dev/walker c 61 0`
2. Insert Kernel Module into the device node
    - Command: `insmod /root/walk.ko`
3. Run Userspace command
    - Command: `./kwalk`
(Or send src/run.sh to Beagle Bone and execute it with `./run.sh`)

Second: Run Object Detection Code (Laptop)
1. Compile Code with C++ and OpenCV flag
    - Command: `` g++ Test5.cpp -o Test5 `pkg-config --cflags --libs opencv4` ``
2. Run Code:
    - Command: `./Laptop`
  
**Expected Results**
- The expected output is the laptop sending real-time data using the Ethernet port to the Beagle Bone. Beagle Bone uses Userspace as a middleman to receive and send data to the kernel.
  The kernel will use the sent data to control the servo motor and the distance sensor
- The object can be changed to be 4 types. First, select the video window, then press the spacebar to change the target combination of object and color. 
