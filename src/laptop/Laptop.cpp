#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <sys/socket.h> 
#include <arpa/inet.h>  
#include <unistd.h>     
#include <sstream>      
#include <cstdlib>      
#include <netinet/tcp.h> // REQUIRED FOR TCP_NODELAY Fix

using namespace cv;
using namespace std;

// --- NETWORK CONFIGURATION ---
#define ROBOT_IP "192.168.7.2" 
#define ROBOT_PORT 5000

// --- VISION CONFIGURATION ---
const int TARGET_AREA_MIN_PIXELS = 1000; 
const int DEAD_ZONE_PIXELS = 50; 
const double SHAPE_APPROX_PRECISION = 0.04; 

const vector<string> TARGET_STATES = {
    "YELLOW_SQUARE",
    "YELLOW_TRIANGLE",
    "BLUE_SQUARE",
    "BLUE_TRIANGLE",
    "STOP_ALL"
};

// --- HSV COLOR THRESHOLDS ---

// YELLOW
const Scalar LOWER_YELLOW(15, 100, 100);
const Scalar UPPER_YELLOW(35, 255, 255);

// DARK BLUE (Target)
const Scalar LOWER_BLUE(90, 60, 30);
const Scalar UPPER_BLUE(140, 255, 255); 

// --- VISION CLASS ---
class RobotVisionSystem {
public: 
    string currentTargetName_;
    Scalar targetLowerColor_;
    Scalar targetUpperColor_;
    string targetShapeName_;
    
    void preprocess_mask(Mat& mask) {
        erode(mask, mask, Mat(), Point(-1, -1), 1);
        dilate(mask, mask, Mat(), Point(-1, -1), 3);
    }
    
    string analyze_shape(const vector<Point>& contour) {
        double perimeter = arcLength(contour, true);
        vector<Point> approx;
        approxPolyDP(contour, approx, SHAPE_APPROX_PRECISION * perimeter, true);
        
        if (approx.size() == 3) return "TRIANGLE";
        if (approx.size() == 4) return "SQUARE"; 
        
        return "UNKNOWN";
    }

    RobotVisionSystem(const string& initialTarget) {
        set_target(initialTarget);
    }
    
    void set_target(const string& targetName) {
        currentTargetName_ = targetName;
        if (targetName == "STOP_ALL") return;

        size_t underscorePos = targetName.find('_');
        string color = targetName.substr(0, underscorePos);
        targetShapeName_ = targetName.substr(underscorePos + 1);

        if (color == "YELLOW") {
            targetLowerColor_ = LOWER_YELLOW;
            targetUpperColor_ = UPPER_YELLOW;
        } else if (color == "BLUE") {
            targetLowerColor_ = LOWER_BLUE;
            targetUpperColor_ = UPPER_BLUE;
        } 
    }

    string get_motor_command(Mat& frame) {
        if (currentTargetName_ == "STOP_ALL") return "STOP:0";
        
        Mat hsv, mask;
        cvtColor(frame, hsv, COLOR_BGR2HSV);
        inRange(hsv, targetLowerColor_, targetUpperColor_, mask);
        preprocess_mask(mask);
        
        imshow("Debug: Target Mask", mask);

        vector<vector<Point>> contours;
        findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        if (contours.empty()) return "STOP:0"; 

        // Find the largest contour
        auto largest_contour_it = max_element(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
            return contourArea(a) < contourArea(b);
        });
        vector<Point> largest_contour = *largest_contour_it;

        if (contourArea(largest_contour) < TARGET_AREA_MIN_PIXELS) return "STOP:0";

        string detected_shape = analyze_shape(largest_contour);

        if (detected_shape == targetShapeName_) {
            Moments M = moments(largest_contour);
            int target_cx = (M.m00 != 0) ? (int)(M.m10 / M.m00) : frame.cols / 2; 

            // Visualization
            Rect bbox = boundingRect(largest_contour);
            rectangle(frame, bbox, Scalar(0, 255, 0), 2); 
            putText(frame, "TARGET: " + currentTargetName_, Point(bbox.tl().x, bbox.tl().y - 10), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2);

            // --- FPV CONTROL LOGIC ---
            int screen_center_x = frame.cols / 2;
            int error = target_cx - screen_center_x;
            int angle = 90; 
            
            line(frame, Point(screen_center_x, 0), Point(screen_center_x, frame.rows), Scalar(255, 255, 255), 1);

            if (error < -DEAD_ZONE_PIXELS) {
                angle = 45; 
                putText(frame, "<< TURN LEFT", Point(10, frame.rows - 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 3);
            } else if (error > DEAD_ZONE_PIXELS) {
                angle = 135; 
                putText(frame, "TURN RIGHT >>", Point(frame.cols - 250, frame.rows - 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 3);
            } else {
                angle = 90; 
                putText(frame, "^ FORWARD ^", Point(screen_center_x - 100, frame.rows - 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 0), 3);
            }
            
            stringstream ss;
            ss << "FORWARD:" << angle;
            return ss.str(); 
        }
        return "STOP:0"; 
    }
};

int main() {
    // --- 0. AUTO-CONFIGURE NETWORK ---
    cout << "Configuring network interface enp5s0..." << endl;
    int ret = system("ifconfig enp5s0 192.168.7.1 netmask 255.255.255.0 up");
    if (ret != 0) {
        cerr << "Warning: Failed to set IP. Are you running with sudo?" << endl;
    } else {
        cout << "Network configured successfully." << endl;
    }

    RobotVisionSystem visionSystem(TARGET_STATES[0]);
    int current_index = 0;
    
    // --- 1. SETUP VIDEO ---
    VideoCapture cap; 
    bool camera_opened = false;
    for (int i = 0; i <= 3; ++i) {
        cap.open(i, CAP_V4L2);
        if (cap.isOpened()) {
            camera_opened = true;
            cout << "Camera opened at index " << i << endl;
            break;
        }
    }
    if (!camera_opened) { cerr << "Camera failed." << endl; return -1; }
    
    cap.set(CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    // --- 2. SETUP NETWORK ---
    int sock = -1;
    struct sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(ROBOT_PORT);
    inet_pton(AF_INET, ROBOT_IP, &serv_addr.sin_addr);

    cout << "Starting Vision Client..." << endl;

    while (true) {
        // A. RECONNECT LOGIC
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_STREAM, 0);
            
            // --- FIX: DISABLE NAGLE'S ALGORITHM (TCP_NODELAY) ---
            // This ensures small packets (commands) are sent IMMEDIATELY
            int flag = 1;
            setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));

            if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
                close(sock);
                sock = -1;
            } else {
                cout << "CONNECTED TO BEAGLEBONE!" << endl;
            }
        }

        // B. VISION PROCESSING
        Mat frame;
        cap >> frame;
        if (frame.empty()) break;
        // flip(frame, frame, 1); // No flip for FPV

        string motor_command = visionSystem.get_motor_command(frame);
        
        // C. SEND COMMAND (If connected)
        if (sock >= 0) {
            // No newline needed if dev_write uses sscanf correctly, but ensure flush
            if (send(sock, motor_command.c_str(), motor_command.length(), 0) < 0) {
                perror("Send failed");
                close(sock);
                sock = -1; 
            }
        }

        // D. DEBUG OUTPUT
        cout << "\rTarget: " << visionSystem.currentTargetName_ << " | Cmd: " << motor_command << " | Net: " << (sock >= 0 ? "OK" : "DISC") << "      " << flush;
        
        putText(frame, "Target: " + visionSystem.currentTargetName_, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(255, 0, 0), 2);
        putText(frame, "Sent: " + motor_command, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
        
        imshow("Vision Client (FPV)", frame);

        int key = waitKey(30);
        if (key == 27) break; 
        if (key == 32) { 
            current_index = (current_index + 1) % TARGET_STATES.size();
            visionSystem.set_target(TARGET_STATES[current_index]);
        }
    }
    
    if (sock >= 0) close(sock);
    return 0;
}
