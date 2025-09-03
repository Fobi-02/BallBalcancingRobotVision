#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <windows.h>
#include <cstdio>
#include <fstream>
#include <conio.h>
#include <vector>
#include "rtfir.hpp"
using namespace cv;
using namespace std;
using namespace chrono;


/*
    _   _   _        _ _           _            
   / \ | |_| |_ _ __(_) |__  _   _| |_ ___  ___ 
  / _ \| __| __| '__| | '_ \| | | | __/ _ \/ __|
 / ___ \ |_| |_| |  | | |_) | |_| | ||  __/\__ \
/_/   \_\__|\__|_|  |_|_.__/ \__,_|\__\___||___/
*/

const float PI = 3.14159265358979323846;
bool Log = true;
HANDLE hSerial;
VideoCapture cap(0);
ofstream log_file("log.csv");
ofstream log_real_pos("log_actual.csv");
double integralX = 0; 
double integralY = 0;
double prevX = 0;
double prevY = 0;
int PrevMotor1 = 331;
int PrevMotor2 = 331;
int PrevMotor3 = 331;
time_point<steady_clock> tStart = steady_clock::now();

double samplerate=25;  // Fs in Hz
double cutoff=7;      // Fc in Hz
int taps=2;           // FIR taps
RTFIR_lowpass lowpassx=RTFIR_lowpass(taps,cutoff/samplerate);
RTFIR_lowpass lowpassy=RTFIR_lowpass(taps,cutoff/samplerate);

/*
 _____                 _   _                 
|  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
|  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/

void connectToRobot()
{
    // starting arduino comunication
    hSerial = CreateFileW(L"\\\\.\\COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    // verifying if the serial port is opened correctly
    if (hSerial == INVALID_HANDLE_VALUE) {
        cerr << "Errore nell'apertura della porta seriale!" << std::endl;
        return;
    }

    // configuring serial port
    DCB serialParams = { 0 };
    serialParams.DCBlength = sizeof(serialParams);
    GetCommState(hSerial, &serialParams);
    serialParams.BaudRate = CBR_115200; // comunication speed (must be the same to Arduino)
    serialParams.ByteSize = 8;
    serialParams.StopBits = ONESTOPBIT;
    serialParams.Parity = NOPARITY;
    SetCommState(hSerial, &serialParams);

    // Timeout
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 1;
    timeouts.ReadTotalTimeoutConstant = 1;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.WriteTotalTimeoutConstant = 0;
    timeouts.WriteTotalTimeoutMultiplier = 0;
    SetCommTimeouts(hSerial, &timeouts);

    // Open webcam
    if(!cap.isOpened()) {
        cout << "Error: Could not open the webcam" << endl;
        return;}
        else {
    cout << "Webcam is open " << endl;}

    // creating the log file
    log_file << "time,pos_x,pos_y,thx,thy,h,target_motor1,target_motor2,target_motor3,speed1,speed2,speed3,thx_original,thy_original" << endl;
    // filw with real position of the motor (sent by arduino)
    log_real_pos << "time,motor1,motor2,motor3" << endl;
    return;
}

int* ballPosition(bool& ballFound, int goalX, int goalY)
{
    // capturing current frame
    Mat frame, undistortedFrame;
    int cx, cy;
    cap >> frame;
    // checking if the frame is empty
    if (frame.empty()) {
        cout << "Error: Could not capture frame" << endl;
    }
    ballFound = 0;
    // Camera calibration matrix (3x3)
    Mat cameraMatrix = (Mat_<double>(3, 3) << 
        6.9092498917251578e+02, 0., 3.1950000000000000e+02,   // Focal length x and image center (cx)
        0., 6.9092498917251578e+02, 2.3950000000000000e+02,   // Focal length y and image center (cy)
        0., 0., 1.);
    // Distortion coefficients (k1, k2, p1, p2, k3)
    Mat distCoeffs = (Mat_<double>(1, 5) << -5.0568739979519028e-01, 3.4056871021111701e-01, 0., 0., -3.3490297480082543e-01);
    undistort(frame, undistortedFrame, cameraMatrix, distCoeffs);
    // Converting image from BGR to HSV
    Mat hsv;
    cvtColor(undistortedFrame, hsv, COLOR_BGR2HSV);
    // Interval for color red in HSV
    // hue scale is 0-179 while saturation and value 0-255
    Scalar lower_red1(int(340*179/360), 50, 10); // low values (Hue, Saturation, Value)
    Scalar upper_red1(int(360*179/360), 255, 255); // high values (Hue, Saturation, Value)
    // i need two intervals to have a mask in the range 330° e 30°
    Scalar lower_red2(int(0*179/360), 50, 10);
    Scalar upper_red2(int(5*179/360), 255, 255);
    // mask to isolate pixels in the correct interval
    Mat mask1, mask2;
    inRange(hsv, lower_red1, upper_red1, mask1);
    inRange(hsv, lower_red2, upper_red2, mask2);
    Mat mask;
    bitwise_or(mask1, mask2, mask);
    // morfologic operations to reduce noise
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    erode(mask, mask, kernel);

    // Finding mask edges
    vector<vector<Point>> contours;
    double areaMinima = 1000.;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        // Contour with bigger area
        vector<Point> largest_contour;
        double max_area = 0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > max_area && area > areaMinima) {
                max_area = area;
                largest_contour = contours[i];
            }
        }

        // Drawing a circle in the goal position
        circle(undistortedFrame, Point(goalY,goalX), 8, Scalar(0, 255, 0), -1);

        // Largest contour
        Moments M = moments(largest_contour);
        if (M.m00 > 0) {
            cx = int(M.m01 / M.m00);
            cy = int(M.m10 / M.m00);

            // Drawing a circle in the center of the ball
            circle(undistortedFrame, Point(cy, cx), 8, Scalar(255, 0, 0), -1);

            // Drawing a rectangle around the ball
            Rect bounding_box = boundingRect(largest_contour);
            rectangle(undistortedFrame, bounding_box, Scalar(0, 255, 0), 2);  // Rettangolo verde
            // Confirming that the ball has been found
            ballFound = 1;
        }
    }
    Mat mask_colored;
    cvtColor(mask, mask_colored, COLOR_GRAY2BGR);  // From gray scale to BGR

    // mask overlay
    Mat overlay;
    undistortedFrame.copyTo(overlay);
    overlay.setTo(Scalar(0, 255, 255), mask);

    // Joining the two images together
    Mat blended;
    addWeighted(undistortedFrame, 0.7, overlay, 0.3, 0, blended);

    // Showing the final result
    imshow("Webcam con Overlay", blended);
    int* out = new int[3];
    if(ballFound == 1) {
        out[0]=cx;
        out[1]=cy;
    }
    else {
        out[0]=0;
        out[1]=0;
    }
    return out;
}

int* motorPosition(double thx, double thy, double h)
{
  // array with the three positions to return
  static int pos[3];
  // robot parameters
  double l = 80.; // length of the robot arms
  double b = 90.; // radius of the basis
  double L = 117.7; // radius of the plate
  // calculating A,B,C values
  double A = -(tan(thx) / sqrt(pow(1/cos(thy), 2) + pow(tan(thx), 2)));
  double B = -(tan(thy) / sqrt(pow(1 / cos(thy), 2) + pow(tan(thx), 2)));
  double C = -(1 / sqrt(pow(1 / cos(thy), 2) + pow(tan(thx), 2)));

  //----------------------------First motor position-----------------------------------------------------------------------------
  // Spherical joint position
  double x1 = abs((L * L * A * C) / sqrt(L * L * A * A * (A * A + C * C)));
  double y1 = 0;
  double z1 = h - (L * L * A * A) / sqrt(L * L * A * A * (A * A + C * C)) * abs(A * C) / (A * C);
  // radiant angle in respect to horizontal
  double th1 = acos((-b + (pow(b, 3) - pow(b, 2) * x1 - b * pow(x1, 2) + pow(x1, 3) - b * pow(y1, 2) + x1 * pow(y1, 2) + b * pow(z1, 2) + x1 * pow(z1, 2) +
              sqrt(-pow(z1, 2) * (pow(b, 4) - 4 * pow(b, 3) * x1 - 4 * pow(l, 2) * (pow(x1, 2) + pow(z1, 2)) + 
              pow(pow(x1, 2) + pow(y1, 2) + pow(z1, 2), 2) - 4 * b * x1 * (-2 * pow(l, 2) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2)) + 
              2 * pow(b, 2) * (-2 * pow(l, 2) + 3 * pow(x1, 2) + pow(y1, 2) + pow(z1, 2))))) / 
              (2 * (pow(b, 2) - 2 * b * x1 + pow(x1, 2) + pow(z1, 2)))) / l);

  //----------------------------Second motor position-----------------------------------------------------------------------------
  double temp = -(C * sqrt(L*L * (pow(A, 8) - 
        8 * sqrt(3) * pow(A, 7) * B -
        216 * sqrt(3) * A * pow(B, 5) * (pow(B, 2) + C * C) -
        24 * sqrt(3) * pow(A, 5) * B * (7 * pow(B, 2) + C * C) +
        4 * pow(A, 6) * (21 * pow(B, 2) + C * C) +
        27 * pow(B, 6) * (3 * pow(B, 2) + 4 * C * C) -
        24 * sqrt(3) * pow(A, 3) * pow(B, 3) * (21 * pow(B, 2) + 10 * C * C) +
        90 * pow(A, 4) * (7 * pow(B, 4) + 2 * pow(B, 2) * C * C) +
        108 * pow(A, 2) * (7 * pow(B, 6) + 5 * pow(B, 4) * C * C))) / (
        (A - sqrt(3) * B) * (pow(A, 4) -
        4 * sqrt(3) * pow(A, 3) * B +
        9 * pow(B, 4) + 12 * pow(B, 2) * C * C -
        4 * sqrt(3) * A * B * (3 * pow(B, 2) + 2 * C * C) +
        2 * pow(A, 2) * (9 * pow(B, 2) + 2 * C * C))));
  double x2 = -abs(temp);
  double y2 =abs((sqrt(3) * C * sqrt(L * L * (pow(A, 8) -
        8 * sqrt(3) * pow(A, 7) * B -
        216 * sqrt(3) * A * pow(B, 5) * (pow(B, 2) + C * C) -
        24 * sqrt(3) * pow(A, 5) * B * (7 * pow(B, 2) + C * C) +
        4 * pow(A, 6) * (21 * pow(B, 2) + C * C) +
        27 * pow(B, 6) * (3 * pow(B, 2) + 4 * C * C) -
        24 * sqrt(3) * pow(A, 3) * pow(B, 3) * (21 * pow(B, 2) + 10 * C * C) +
        90 * pow(A, 4) * (7 * pow(B, 4) + 2 * pow(B, 2) * C * C) +
        108 * pow(A, 2) * (7 * pow(B, 6) + 5 * pow(B, 4) * C * C)))) / (
        (A - sqrt(3) * B) * (pow(A, 4) -
        4 * sqrt(3) * pow(A, 3) * B +
        9 * pow(B, 4) + 12 * pow(B, 2) * C * C -
        4 * sqrt(3) * A * B * (3 * pow(B, 2) + 2 * C * C) +
        2 * pow(A, 2) * (9 * pow(B, 2) + 2 * C * C))));
  double z2 = h - ((sqrt(L * L * (pow(A, 8) -
        8 * sqrt(3) * pow(A, 7) * B -
        216 * sqrt(3) * A * pow(B, 5) * (pow(B, 2) + C * C) -
        24 * sqrt(3) * pow(A, 5) * B * (7 * pow(B, 2) + C * C) +
        4 * pow(A, 6) * (21 * pow(B, 2) + C * C) +
        27 * pow(B, 6) * (3 * pow(B, 2) + 4 * C * C) -
        24 * sqrt(3) * pow(A, 3) * pow(B, 3) * (21 * pow(B, 2) + 10 * C * C) +
        90 * pow(A, 4) * (7 * pow(B, 4) + 2 * pow(B, 2) * C * C) +
        108 * pow(A, 2) * (7 * pow(B, 6) + 5 * pow(B, 4) * C * C))) / (pow(A, 4) -
        4 * sqrt(3) * pow(A, 3) * B +
        9 * pow(B, 4) + 12 * pow(B, 2) * C * C -
        4 * sqrt(3) * A * B * (3 * pow(B, 2) + 2 * C * C) +
        2 * pow(A, 2) * (9 * pow(B, 2) + 2 * C * C))))*temp/x2*-1;
  double th2 = acos((-b + sqrt((pow((2*b + x2 - sqrt(3)*y2)*(pow(b,2) - pow(x2,2) - pow(y2,2)) + (2*b - x2 + sqrt(3)*y2)*pow(z2,2) + 
            2*sqrt(-(pow(z2,2)*(pow(b,4) + 2*pow(b,3)*(x2 - sqrt(3)*y2) + pow(pow(x2,2) + pow(y2,2) + pow(z2,2),2) + 
            2*b*(x2 - sqrt(3)*y2)*(-2*pow(l,2) + pow(x2,2) + pow(y2,2) + pow(z2,2)) + 
            pow(b,2)*(-4*pow(l,2) + 3*pow(x2,2) - 2*sqrt(3)*x2*y2 + 5*pow(y2,2) + 2*pow(z2,2)) - 
            pow(l,2)*(pow(x2,2) - 2*sqrt(3)*x2*y2 + 3*pow(y2,2) + 4*pow(z2,2))))),2) + 
            pow((2*sqrt(3)*b + sqrt(3)*x2 - 3*y2)*(pow(b,2) - pow(x2,2) - pow(y2,2)) + (2*sqrt(3)*b - sqrt(3)*x2 + 3*y2)*pow(z2,2) + 
            2*sqrt(3)*sqrt(-(pow(z2,2)*(pow(b,4) + 2*pow(b,3)*(x2 - sqrt(3)*y2) + pow(pow(x2,2) + pow(y2,2) + pow(z2,2),2) + 
            2*b*(x2 - sqrt(3)*y2)*(-2*pow(l,2) + pow(x2,2) + pow(y2,2) + pow(z2,2)) + 
            pow(b,2)*(-4*pow(l,2) + 3*pow(x2,2) - 2*sqrt(3)*x2*y2 + 5*pow(y2,2) + 2*pow(z2,2)) - 
            pow(l,2)*(pow(x2,2) - 2*sqrt(3)*x2*y2 + 3*pow(y2,2) + 4*pow(z2,2))))),2))/
            pow(4*pow(b,2) + pow(x2,2) - 2*sqrt(3)*x2*y2 + 3*pow(y2,2) + 4*b*(x2 - sqrt(3)*y2) + 4*pow(z2,2),2))/2.)/l);

  //----------------------------Third motor position-----------------------------------------------------------------------------
  double temp2 = -(C * sqrt(L * L * (pow(A, 8) + 8 * sqrt(3) * pow(A, 7) * B + 
      216 * sqrt(3) * A * pow(B, 5) * (pow(B, 2) + pow(C, 2)) + 
      24 * sqrt(3) * pow(A, 5) * B * (7 * pow(B, 2) + pow(C, 2)) + 
      4 * pow(A, 6) * (21 * pow(B, 2) + pow(C, 2)) + 
      27 * pow(B, 6) * (3 * pow(B, 2) + 4 * pow(C, 2)) + 
      24 * sqrt(3) * pow(A, 3) * pow(B, 3) * (21 * pow(B, 2) + 10 * pow(C, 2)) + 
      90 * pow(A, 4) * (7 * pow(B, 4) + 2 * pow(B, 2) * pow(C, 2)) + 
      108 * pow(A, 2) * (7 * pow(B, 6) + 5 * pow(B, 4) * pow(C, 2)))) / ((A + sqrt(3) * B) * (pow(A, 4) + 
      4 * sqrt(3) * pow(A, 3) * B + 9 * pow(B, 4) + 12 * pow(B, 2) * pow(C, 2) + 
      4 * sqrt(3) * A * B * (3 * pow(B, 2) + 2 * pow(C, 2)) + 
      2 * pow(A, 2) * (9 * pow(B, 2) + 2 * pow(C, 2)))));
  double x3 = -abs(temp2);
  double y3 = -abs(-(sqrt(3) * C * sqrt(L * L * (pow(A, 8) + 8 * sqrt(3) * pow(A, 7) * B + 
      216 * sqrt(3) * A * pow(B, 5) * (pow(B, 2) + pow(C, 2)) + 
      24 * sqrt(3) * pow(A, 5) * B * (7 * pow(B, 2) + pow(C, 2)) + 
      4 * pow(A, 6) * (21 * pow(B, 2) + pow(C, 2)) + 
      27 * pow(B, 6) * (3 * pow(B, 2) + 4 * pow(C, 2)) + 
      24 * sqrt(3) * pow(A, 3) * pow(B, 3) * (21 * pow(B, 2) + 10 * pow(C, 2)) + 
      90 * pow(A, 4) * (7 * pow(B, 4) + 2 * pow(B, 2) * pow(C, 2)) + 
      108 * pow(A, 2) * (7 * pow(B, 6) + 5 * pow(B, 4) * pow(C, 2)))) / ((A + sqrt(3) * B) * (pow(A, 4) + 
      4 * sqrt(3) * pow(A, 3) * B + 9 * pow(B, 4) + 12 * pow(B, 2) * pow(C, 2) + 
      4 * sqrt(3) * A * B * (3 * pow(B, 2) + 2 * pow(C, 2)) + 2 * pow(A, 2) * (9 * pow(B, 2) + 2 * pow(C, 2))))));
  double z3 = h - (sqrt(pow(L,2)*(pow(A,8) + 8*sqrt(3)*pow(A,7)*B + 216*sqrt(3)*A*pow(B,5)*(pow(B,2) + pow(C,2)) + 24*sqrt(3)*pow(A,5)*B*(7*pow(B,2) + pow(C,2)) + 
        4*pow(A,6)*(21*pow(B,2) + pow(C,2)) + 27*pow(B,6)*(3*pow(B,2) + 4*pow(C,2)) + 24*sqrt(3)*pow(A,3)*pow(B,3)*(21*pow(B,2) + 10*pow(C,2)) + 
        90*pow(A,4)*(7*pow(B,4) + 2*pow(B,2)*pow(C,2)) + 108*pow(A,2)*(7*pow(B,6) + 5*pow(B,4)*pow(C,2))))/
        (pow(A,4) + 4*sqrt(3)*pow(A,3)*B + 9*pow(B,4) + 12*pow(B,2)*pow(C,2) + 4*sqrt(3)*A*B*(3*pow(B,2) + 2*pow(C,2)) + 2*pow(A,2)*(9*pow(B,2) + 2*pow(C,2))))*abs(temp2)/temp2;
  double th3 = acos((-b + sqrt((pow((2*b + x3 + sqrt(3)*y3)*(pow(b,2) - pow(x3,2) - pow(y3,2)) - (-2*b + x3 + sqrt(3)*y3)*pow(z3,2) + 
            2*sqrt(-(pow(z3,2)*(pow(b,4) + 2*pow(b,3)*(x3 + sqrt(3)*y3) + pow(pow(x3,2) + pow(y3,2) + pow(z3,2),2) + 
            2*b*(x3 + sqrt(3)*y3)*(-2*pow(l,2) + pow(x3,2) + pow(y3,2) + pow(z3,2)) + 
            pow(b,2)*(-4*pow(l,2) + 3*pow(x3,2) + 2*sqrt(3)*x3*y3 + 5*pow(y3,2) + 2*pow(z3,2)) - 
            pow(l,2)*(pow(x3,2) + 2*sqrt(3)*x3*y3 + 3*pow(y3,2) + 4*pow(z3,2))))),2) + 
            pow((2*sqrt(3)*b + sqrt(3)*x3 + 3*y3)*(pow(b,2) - pow(x3,2) - pow(y3,2)) + (2*sqrt(3)*b - sqrt(3)*x3 - 3*y3)*pow(z3,2) + 
            2*sqrt(3)*sqrt(-(pow(z3,2)*(pow(b,4) + 2*pow(b,3)*(x3 + sqrt(3)*y3) + pow(pow(x3,2) + pow(y3,2) + pow(z3,2),2) + 
            2*b*(x3 + sqrt(3)*y3)*(-2*pow(l,2) + pow(x3,2) + pow(y3,2) + pow(z3,2)) + 
            pow(b,2)*(-4*pow(l,2) + 3*pow(x3,2) + 2*sqrt(3)*x3*y3 + 5*pow(y3,2) + 2*pow(z3,2)) - 
            pow(l,2)*(pow(x3,2) + 2*sqrt(3)*x3*y3 + 3*pow(y3,2) + 4*pow(z3,2))))),2))/
            pow(4*pow(b,2) + pow(x3,2) + 2*sqrt(3)*x3*y3 + 3*pow(y3,2) + 4*b*(x3 + sqrt(3)*y3) + 4*pow(z3,2),2))/2.)/l);

  // convering the results in number of motor steps
  pos[0] = th1*16*100/PI;
  pos[1] = th2*16*100/PI;
  pos[2] = th3*16*100/PI;
  return pos;
}

double PID(int pos, int goal, double& integral, double& previous, int Dt)
{
    double kP=0.00035;
    double kI=0.000015;
    double kD=0.17;

    double th;
    // error is the difference from ball position and goal
    int err = pos-goal;

    // proportional term
    double P = kP*err;
    
    // integral term
    integral += err;
    double I = kI*integral;

    // derivative term
    double D = kD*(err-previous)/Dt;

    previous = err;
    th = P + I + D;
    return th;
}

string receiveData()
{
    char buffer[256];
    DWORD bytesRead;
    ReadFile(hSerial, buffer, sizeof(buffer)-1, &bytesRead, NULL);
    return std::string(buffer, bytesRead);
}

void sendData(const string& data)
{
    DWORD bytesWritten;
    WriteFile(hSerial, data.c_str(), data.size(), &bytesWritten, NULL);
}

BOOL WINAPI ConsoleHandler(DWORD signal) {
    switch (signal) {
        case CTRL_C_EVENT:
            std::cout << "\nProgram is closing" << std::endl;
            sendData("0_0_0_500_500_500\n");
            return FALSE;
        case CTRL_BREAK_EVENT:
            std::cout << "\nHai premuto Ctrl+Break!" << std::endl;
            return FALSE;
        case CTRL_CLOSE_EVENT:
            std::cout << "\nLa finestra console è stata chiusa!" << std::endl;
            return FALSE;
        case CTRL_LOGOFF_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            std::cout << "\nLogoff o spegnimento rilevato!" << std::endl;
            return FALSE;
        default:
            return FALSE;
    }
}

void limit(double &thx, double& thy, double& h)
{
    // limiting thx and thy to [-0.25,0.25]
    if(thx>0.25)
        thx = 0.25;
    else if(thx < -0.25)
        thx = -0.25;
    if(thy>0.25)
        thy = 0.25;
    else if(thy < -0.25)
        thy = -0.25;

    // if the heigth of the platform is above the permitted one (defined by three planes) i translate it down under the planes
    // i also move by an additional 1mm to be more conservative

    if( h > -151.521*(-1-0.695756*thx))
        h = -151.521*(-1-0.695756*thx) - 3;
    if( h > -146.025*(-1+0.274268*thx-0.514345*thy))
        h = -146.025*(-1+0.274268*thx-0.514345*thy) - 3;
    if( h > -146.025*(-1+0.274268*thx+0.514345*thy))
        h = -146.025*(-1+0.274268*thx+0.514345*thy) - 3;
}

void controlLoop(time_point<steady_clock> tStart, int goalX, int goalY, bool& ballFound, time_point<steady_clock>& previousTime, int* speed) {   
    // keeping track of actual time
        auto tEnd = steady_clock::now();
        auto time = duration_cast<milliseconds>(tEnd - tStart);
        if(Log) log_file << time.count() << ",";

        //step 1: calculating ball position
        int* ballPos = ballPosition(ballFound,goalX,goalY);

        // position of the robot at rest
        double thx = 0.0001;
        double thy = 0.0001;
        double h = 120;

        auto Dt = duration_cast<milliseconds>(tEnd - previousTime);
        int dt = static_cast<int>(Dt.count());
        if(ballFound==1) {
            //step 2: calculating thx and thy with PID controller
            thx = PID(ballPos[0], goalX, integralX, prevX, dt);
            thy = -PID(ballPos[1], goalY, integralY, prevY, dt);
            h = 120;
            // protecting the system from divisions by 0
            if(thx==0)
                thx=0.0001;
            if(thy==0)
                thy=0.0001;

            // limiting motor positions
            limit(thx, thy, h);
        }

        // updating previous time
        previousTime = tEnd;

        //filtering the signal
        double thx_original=thx;
        double thy_original=thy;
        thx = lowpassx.Filter(thx);
        thy = lowpassy.Filter(thy);

        //step 3: calculating motor goal position
        int* pos = motorPosition(thx, thy, h);

        // speed of the motor proportional to the difference in position
        if(ballFound==1) {
            speed[0]=min(500/17*abs(pos[0]-PrevMotor1),700);
            speed[1]=min(500/17*abs(pos[1]-PrevMotor2),700);
            speed[2]=min(500/17*abs(pos[2]-PrevMotor3),700);
        } else {
            speed[0]=500;
            speed[1]=500;
            speed[2]=500;
        }

        PrevMotor1 = pos[0];
        PrevMotor2 = pos[1];
        PrevMotor3 = pos[2];

        //step 4: sending the data trough serial port
        string to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' 
        + to_string(speed[0]) + '_' + to_string(speed[1]) +'_' + to_string(speed[2]) + '\n';
        sendData(to_send);

        if(Log) log_file << ballPos[0] << "," << ballPos[1] << "," << thx << "," << thy << "," << h << "," << 
        pos[0] << "," << pos[1] << "," << pos[2] << "," << speed[0] << "," << speed[1] << "," << speed[2] << "," <<
        thx_original<<","<<thy_original<< endl;
        // eliminating dynamic memory
        //delete[] pos;
        if(Log) log_real_pos << receiveData();
}

/*
  ____                                          _     
 / ___|___  _ __ ___  _ __ ___   __ _ _ __   __| |___ 
| |   / _ \| '_ ` _ \| '_ ` _ \ / _` | '_ \ / _` / __|
| |__| (_) | | | | | | | | | | | (_| | | | | (_| \__ \
 \____\___/|_| |_| |_|_| |_| |_|\__,_|_| |_|\__,_|___/
*/

void balance(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    time_point<steady_clock> previousTime = tStart;
    // preparing integrative term for PID
    integralX = 0; integralY = 0;
    // saving previous position of the ball
    prevX = 0; prevY = 0;
    bool ballFound;
    // webcam parameter for center of the image
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    int speed[3];
    speed[0]=500;
    speed[1]=500;
    speed[2]=500;

    // cycle for every frame
    while(true) {
        controlLoop(tStart,cx,cy,ballFound,previousTime,speed);
        
        if(_kbhit())
            break;

        if(waitKey(1) >= 0) {
            break;
        }
    }
}

void circle(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto previousTime = tStart;
    // preparing integrative term for PID
    double integralX = 0; double integralY = 0;
    // saving previous position of the ball
    double prevX = 0; double prevY = 0;
    bool ballFound;
    // parameters of the circle
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    double r = 140;
    double theta = 0;
    int speed[3];
    speed[0]=500;
    speed[1]=500;
    speed[2]=500;

    while(true) {
        theta += 2*2*PI/180;
        controlLoop(tStart,cx+r*cos(theta),cy+r*sin(theta),ballFound,previousTime,speed);
        if(_kbhit())
            break;
        
        if(waitKey(1) >= 0) {
            break;
        }
    }
}

void star(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto tStart = steady_clock::now();
    auto previousTime = tStart;
    //preparo il termine integrativo per il PID
    double integralX = 0; double integralY = 0;
    //salvo il valore della posizione precedente della pallina
    double prevX = 0; double prevY = 0;
    bool ballFound;
    //calcolo i parametri della webcam e il centro
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    double r = 0;
    double Rmin = 80;
    double Rmax = 160;
    int n = 5;
    int p = 2;
    double theta = 0;
    int speed[3];
    speed[0]=500;
    speed[1]=500;
    speed[2]=500;

    //ciclo per catturare ogni frame
    while(true) {
        theta += 2*PI/180;
        r = Rmin + (Rmax-Rmin)*pow((1+cos(n*theta))/2,p);
        controlLoop(tStart,cx+r*cos(theta),cy+r*sin(theta),ballFound,previousTime,speed);
        if(_kbhit())
            break;
        
        if(waitKey(1) >= 0) {
            break;
        }
    }
}

void square(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto tStart = steady_clock::now();
    auto previousTime = tStart;
    //preparo il termine integrativo per il PID
    double integralX = 0; double integralY = 0;
    //salvo il valore della posizione precedente della pallina
    double prevX = 0; double prevY = 0;
    bool ballFound;
    //calcolo i parametri della webcam e il centro
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    double r = 0;
    double a = 160;
    double theta = 0;
    int speed[3];
    speed[0]=500;
    speed[1]=500;
    speed[2]=500;

    //ciclo per catturare ogni frame
    while(true) {
        theta += 2*2*PI/180;
        r = a/(max(abs(cos(theta)),abs(sin(theta))));
        controlLoop(tStart,cx+r*cos(theta),cy+r*sin(theta),ballFound,previousTime,speed);
        if(_kbhit())
            break;
        
        if(waitKey(1) >= 0) {
            break;
        }
    }
}
/*
void jump(HANDLE hSerial)
{
    // jump sequence
    this_thread::sleep_for(seconds(2));
    int speed = 200;
    int* pos = motorPosition(0.0001, 0.0001, 70);
    string to_send;
    to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
    sendData(to_send);
    cout << to_send << endl;
    this_thread::sleep_for(seconds(6));
    speed = 2000;
    pos = motorPosition(0.0001, 0.0001, 130);
    to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
    sendData(to_send);
    cout << to_send << endl;
}

*/

/*
 __  __       _       
|  \/  | __ _(_)_ __  
| |\/| |/ _` | | '_ \ 
| |  | | (_| | | | | |
|_|  |_|\__,_|_|_| |_|
*/

int main()
{
    if (!SetConsoleCtrlHandler(ConsoleHandler, TRUE)) {
        std::cerr << "Errore nella registrazione dell'handler." << std::endl;
        return 1;
    }
    connectToRobot();

    while(true) {
        cout << "Comandi disponibili: " <<  "b " << "per bilanciare la pallina, " << "j " << "per saltare, " <<
             "per fare una figura c,s,q (cerchio, stella, quadrato)." << endl;
        char comando;
        cin >> comando;

        if(comando == 'b') {
            balance(hSerial,cap,log_file,log_real_pos);
        } else if(comando == 'c') {
            circle(hSerial,cap,log_file,log_real_pos);
        } else if(comando == 's') {
            star(hSerial,cap,log_file,log_real_pos);
        } else if(comando == 'q') {
            square(hSerial,cap,log_file,log_real_pos);
        }
    }
    // closing the webcam
    cap.release();
    destroyWindow("Webcam con Overlay");
    // closing serial port
    CloseHandle(hSerial);
    log_file.close();
    return 0;
}




// to compile:
//  mingw32-make
//  ./ProvaOpenCV.exe