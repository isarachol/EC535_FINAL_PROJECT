// ==========================================
// libraries
// ==========================================
#include <opencv2/opencv.hpp> // OpenCV library: Provide computer vision and image processing 
#include <iostream> // Standard C++ input and output stream library
#include <vector> // Vector Container: for dynamic arrary 
#include <string>
#include <algorithm>
#include <cmath>
#include <sys/socket.h> // For socket Programming 
#include <arpa/inet.h> // Internet Address Operations
#include <unistd.h>     
#include <sstream>      
#include <cstdlib>      
#include <netinet/tcp.h> // TCP protocol specific options - enables TCP_NODELAY

//======================================
// Namespace - avoid need to use prefix 
//======================================
using namespace cv;
using namespace std;

//======================================
// Network Configuration
//======================================
// This is the BeagleBone IP address, which can vary based on each device
// Using ifconfig in BeagleBone to correct the IP address
#define ROBOT_IP "192.168.7.2" 
// Define port number: 
// The BeagleBone server should have the same for a successful connection 
#define ROBOT_PORT 5000

//======================================
// Vision Configuration
//======================================
// Minimum contour area in pixels to be considered a valid object
// smaller object will be filtered out
const int Min_pixel = 1000; 

// Dead zone of center alignment (in pixels)
// The variable determines if the robot will go forward, left, or right
// If the verified object is outside the dead zone, the  robot will turn left or right to make the Target_state object 
// within the defined zone
const int Dead_zone = 50; 

// This is the shape approximation coefficient
// It will be used in approxPolyPD to simplify the contour
// Higher value leads to aggressive simplification, having a higher chance of missing the shape 
const double Shape_approx = 0.04; 

// List of target states the robot can track 
// The target can be changed by pressing SPACE on the keyboard
const vector<string> Target_state = {
    "YELLOW_SQUARE",
    "YELLOW_TRIANGLE",
    "BLUE_SQUARE",
    "BLUE_TRIANGLE",
    "STOP_ALL"
};

//======================================
// HSV Color Thresholds 
//======================================
// YELLOW
const Scalar Lower_yellow(15, 100, 100);
const Scalar Upper_yellow(35, 255, 255);

// DARK BLUE
const Scalar Lower_blue(90, 60, 30);
const Scalar Upper_blue(140, 255, 255); 

//======================================
// Vision Class
//======================================
// The class handles object detection, tracking, and motor command generation 
class RobotVisionSystem {
public:
    // Member Variables
    string currentTarget_stateName_; // Current Tracking Target 
    Scalar Target_stateLowerColor_; // Lower HSV threshold for target color
    Scalar Target_stateUpperColor_; // Upper HSV threshold for target color
    string Target_stateShapeName_; // Current target shape 

    // This is the noise filtering process for binary masking
    void preprocess_mask(Mat& mask) {
        // Erode: This will remove small white noise pixels (1 iteration)
        erode(mask, mask, Mat(), Point(-1, -1), 1);
        // Dilate: This will restore object size and fill small holes (3 iterations)
        // More dilate than erode helps connect broken object regions
        dilate(mask, mask, Mat(), Point(-1, -1), 3);
    }

    // This function will analyze the contour 
    // Determine whether it is a square or a triangle 
    string analyze_shape(const vector<Point>& contour) {
        //Calculate the perimeter of the contour 
        double perimeter = arcLength(contour, true);

        // This will approximate the contour to a simpler polygon 
        vector<Point> approx;
        approxPolyDP(contour, approx, Shape_approx * perimeter, true);

        // IF conner = 3 then it is a triangle 
        // IF conner = 4, then it is a square 
        if (approx.size() == 3) return "TRIANGLE";
        if (approx.size() == 4) return "SQUARE"; 

        // Other shape will be determined as unknown
        return "UNKNOWN";
    }

    // Initializes the vision system with a starting target 
    RobotVisionSystem(const string& initialTarget_state) {
        set_Target_state(initialTarget_state);
    }

    // The function will update the target's shape and color based on the target name
    void set_Target_state(const string& Target_stateName) {
        currentTarget_stateName_ = Target_stateName;
        if (Target_stateName == "STOP_ALL") return;

        // The function is parsing to distinguish between the shape and color 
        // Format: "shape_color"
        size_t underscorePos = Target_stateName.find('_'); // Search for underscore and return position 
        string color = Target_stateName.substr(0, underscorePos); // Extract the shape (first to underscore)
        Target_stateShapeName_ = Target_stateName.substr(underscorePos + 1); // Extract the color (from underscore to end)

        // Set Lower and Upper HSV of target based on the selected color
        if (color == "YELLOW") {
            Target_stateLowerColor_ = Lower_yellow;
            Target_stateUpperColor_ = Upper_yellow;
        } else if (color == "BLUE") {
            Target_stateLowerColor_ = Lower_blue;
            Target_stateUpperColor_ = Upper_blue;
        } 
    }

    // This function will process the frame and return the motor command string 
    string get_motor_command(Mat& frame) {
        if (currentTarget_stateName_ == "STOP_ALL") return "STOP:0"; 

        // Step 1: Color Filtering 
        // The function converts BGR to HSV format, leading to better color detection
        Mat hsv, mask; // Matrix for HSV and Mask
        cvtColor(frame, hsv, COLOR_BGR2HSV); // Convert BGR to HSV and store in matrix HSV
        inRange(hsv, Target_stateLowerColor_, Target_stateUpperColor_, mask); // Create binary mask: white pixels = target color, black = everything else
        preprocess_mask(mask); // Filtering out the noise

        // Show Mask after filtering the noise
        imshow("Debug: Target_state Mask", mask);


        // Step 2: Contour Detection 
        // The function will find the object boundaries in the mask (white mask)
        vector<vector<Point>> contours; // Container of vector of vector of point to handle multiple objects in the frame
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE); // Find contour of all object 

        // No object is found -> Send Stop command
        if (contours.empty()) return "STOP:0"; 

        // Step 3: Find the largest Contour 
        // The line will try to find the largest white mask detected in the frame 
        auto largest_contour_it = max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
            return contourArea(a) < contourArea(b);
        });
        vector<Point> largest_contour = *largest_contour_it; // Store the largest into the vector 

        // Compare the largest object is smaller than the threshold 
        // The value of the minimum pixel comes from the experiments
        // The value can vary based on the target object and application
        if (contourArea(largest_contour) < Min_pixel) return "STOP:0";

        // The object will be analyzed to verify the shape
        // Target shape: Triangle and Square 
        string detected_shape = analyze_shape(largest_contour);

        // Step 5: Find the center of the object (center of mass)
        if (detected_shape == Target_stateShapeName_) {
            Moments M = moments(largest_contour); // Reurn the sum of x-coordinate 
            // Find the center of the x-coordinate on the object
            // Return the x-coordinate of the object's center
            int Target_state_cx = (M.m00 != 0) ? (int)(M.m10 / M.m00) : frame.cols / 2; 

            // Visualization
            // Draw a bounding box around the detected object
            Rect bbox = boundingRect(largest_contour);
            rectangle(frame, bbox, Scalar(0, 255, 0), 2); 
            putText(frame, "Target_state: " + currentTarget_stateName_, Point(bbox.tl().x, bbox.tl().y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);

            //  Step 6: FPV Control Logic
            // This will calculate how much the x of the object's center is off from the screen's center
            int screen_center_x = frame.cols / 2;
            int error = Target_state_cx - screen_center_x;
            int angle = 90; 

            // Draw the vertical line for reference at the center of the screen
            line(frame, Point(screen_center_x, 0), Point(screen_center_x, frame.rows), Scalar(255, 255, 255), 1);

            // Compare the error of the predefined value
            // It will send a steering command based on object position 
            if (error < -Dead_zone) {
                angle = 45; 
                putText(frame, "<< TURN LEFT", Point(10, frame.rows - 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 3);
            } else if (error > Dead_zone) {
                angle = 135; 
                putText(frame, "TURN RIGHT >>", Point(frame.cols - 250, frame.rows - 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 3);
            } else {
                angle = 90; 
                putText(frame, "^ FORWARD ^", Point(screen_center_x - 100, frame.rows - 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 3);
            }

            // Set the command's format
            // Format command string: "FORWARD:angle"
            stringstream ss;
            ss << "FORWARD:" << angle;
            return ss.str(); 
        }
        return "STOP:0"; 
    }
};

int main() {
    //==========================================================================
    // Network Interface Configuration 
    // Step 1: Sets up the network interface to communicate with the Beagle Bone 
    //==========================================================================
    cout << "Configuring network interface enp5s0..." << endl;

    // Execute shell command to configure network:
    // - enp5s0: Network interface name -> This name varies based on each laptop or desktop 
    // - 192.168.7.1: This computer's IP address -> IP address varies based on the server (192.168.7.x) -> need to be change 
    // - netmask 255.255.255.0: Network mask (allows 192.168.7.0-255)
    // - up: Activate the interface
    int ret = system("ifconfig enp5s0 192.168.7.1 netmask 255.255.255.0 up");
    if (ret != 0) {
        cerr << "Warning: Failed to set IP. Are you running with sudo?" << endl;
    } else {
        cout << "Network configured successfully." << endl;
    }

    // ===========================================================
    // Initializes Vision System
    // ===========================================================
    RobotVisionSystem visionSystem(Target_state[0]);
    int current_index = 0;
    
    // Step 1: Set up Video Capture and Access the camera 
    VideoCapture cap; // OpenCV video capture object
    bool camera_opened = false;

    // Try to open the camera at indices 0-3
    // Different systems number cameras differently, so it needs to be changed based on each system
    for (int i = 0; i <= 3; ++i) {
        cap.open(i, CAP_V4L2); // CAP_V4L2 = Video4Linux2 driver (for Linux)
        if (cap.isOpened()) {
            camera_opened = true;
            cout << "Camera opened at index " << i << endl;
            break;
        }
    }
    if (!camera_opened) { cerr << "Camera failed." << endl; return -1; }

    // Set Camera Resolution 
    // Can be changed, it does not affect object detection 
    // For user reference to see what the robot sees
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    //===========================================
    // 2. SETUP NETWORK CONNECTION
    //===========================================
    // Socket for TCP connection to robot
    int sock = -1; // -1 mean not connected
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;  // IPv4 protocol
    serv_addr.sin_port = htons(ROBOT_PORT); // Port number (converted to network byte order)
    inet_pton(AF_INET, ROBOT_IP, &serv_addr.sin_addr); // Convert IP string to binary format

    cout << "Starting Vision Client..." << endl;

    //================================================
    // Main loop
    //================================================
    while (true) {
        //===================================================
        // Connect Logic
        //===================================================
        // If not connected, attempt to connect to the robot
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_STREAM, 0); // Create TCP socket
            
            // Performance Optimization 
            // The issue is the latency of command sending through the Ethernet Port
            // This disables Nagle's algorithm (TCP_NODELAY)
            // Nagle's algorithm delays small packets to batch them together
            //Disabling this algorithm results in the command being sent immediately 
            int flag = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

            // Attempt to connect to the robot
            if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                close(sock);
                sock = -1;
            } else {
                cout << "CONNECTED TO BEAGLEBONE!" << endl;
            }
        }

        //==================================================
        // Vision Processing
        //==================================================
        // This will try to capture one frame from the camera. 
        // Then it will format and send the command to the robot 
        Mat frame; // Create frame matrix 
        cap >> frame; // Capture one frame
        if (frame.empty()) break; 

        // Get steering command
        string motor_command = visionSystem.get_motor_command(frame);
        
        // Send command to the robot 
        // The command is sent only if they are connected
        if (sock >= 0) {
            // Send command string over TCP socket
            // motor_command.c_str() converts C++ string to C-style char array
            // motor_command.length() is the number of bytes to send
            if (send(sock, motor_command.c_str(), motor_command.length(), 0) < 0) {
                perror("Send failed");
                close(sock);
                sock = -1; 
            }
        }

        // Debug Output
        // Print status to console 
        cout << "\rTarget_state: " << visionSystem.currentTarget_stateName_ << " | Cmd: " << motor_command << " | Net: " << (sock >= 0 ? "OK" : "DISC") << "      " << flush;

        // ===============================
        // User Visualization 
        //================================
        // It will overlay the text on the screen 
        //Consists of: current target and commands that are sent 
        putText(frame, "Target_state: " + visionSystem.currentTarget_stateName_, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
        putText(frame, "Sent: " + motor_command, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);

        // Show the Video
        imshow("Vision Client (FPV)", frame);

        // Target Changing y=using keyboard
        // It will wait 30 ms for keyboard input. -> This also determines FPS, whichis  approximately 33 FPS
        int key = waitKey(30);

        // ESC will stop the process 
        if (key == 27) break; 

        // Spacebar will change the target object 
        if (key == 32) { 
            current_index = (current_index + 1) % Target_state.size();
            visionSystem.set_Target_state(Target_state[current_index]);
        }
    }

    // Clean the socket 
    // It will close the socket if they are connected
    if (sock >= 0) close(sock);
    return 0;
}
