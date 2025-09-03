#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <windows.h>
#include <cstdio>
#include <fstream>
#include <conio.h>
using namespace cv;
using namespace std;
using namespace chrono;
const float PI = 3.14159265358979323846;

bool Log = true;

/*
 _____                 _   _                 
|  ___|   _ _ __   ___| |_(_) ___  _ __  ___ 
| |_ | | | | '_ \ / __| __| |/ _ \| '_ \/ __|
|  _|| |_| | | | | (__| |_| | (_) | | | \__ \
|_|   \__,_|_| |_|\___|\__|_|\___/|_| |_|___/
*/

void connectToRobot(HANDLE& hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos)
{
    //inizio la comunicazione con arduino
    hSerial = CreateFileW(L"\\\\.\\COM3", GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    //verifica se la porta è stata aperta correttamente
    if (hSerial == INVALID_HANDLE_VALUE) {
        cerr << "Errore nell'apertura della porta seriale!" << std::endl;
        return;
    }

    // Configura la porta seriale
    DCB serialParams = { 0 };
    serialParams.DCBlength = sizeof(serialParams);
    GetCommState(hSerial, &serialParams);
    serialParams.BaudRate = CBR_115200; // Velocità della comunicazione (deve corrispondere a quella di Arduino)
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

    // Apre la webcam
    if(!cap.isOpened()) {
        cout << "Error: Could not open the webcam" << endl;
        return;}

    //creo un file di log
    log_file << "time,pos_x,pos_y,thx,thy,h,target_motor1,target_motor2,target_motor3" << endl;
    // file con la posizione reale dei tre motori
    log_real_pos << "time,motor1,motor2,motor3" << endl;
    return;
}

int* ballPosition(VideoCapture cap, bool& ball_found, int goalX, int goalY)
{
    // cattura il frame attuale
    Mat frame, undistortedFrame;
    int cx, cy;
    cap >> frame;
    // controlla se il frame è vuoto
    if (frame.empty()) {
        cout << "Error: Could not capture frame" << endl;
    }
    ball_found = 0;
    // Matrice intrinseca della camera (3x3)
    Mat cameraMatrix = (Mat_<double>(3, 3) << 
        6.9092498917251578e+02, 0., 3.1950000000000000e+02,   // Focale in x e centro dell'immagine (cx)
        0., 6.9092498917251578e+02, 2.3950000000000000e+02,   // Focale in y e centro dell'immagine (cy)
        0., 0., 1.);
    // Coefficienti di distorsione (k1, k2, p1, p2, k3)
    Mat distCoeffs = (Mat_<double>(1, 5) << -5.0568739979519028e-01, 3.4056871021111701e-01, 0., 0., -3.3490297480082543e-01);
    undistort(frame, undistortedFrame, cameraMatrix, distCoeffs);
    // Converti il frame da BGR a HSV
    Mat hsv;
    cvtColor(undistortedFrame, hsv, COLOR_BGR2HSV);
    // Definisci l'intervallo per il colore arancione in HSV
    //hue è su una scala 0-179 mentre saturation e value su una scala 0-255
    Scalar lower_red1(int(340*179/360), 50, 10); // valori bassi (Hue, Saturation, Value)
    Scalar upper_red1(int(360*179/360), 255, 255); // valori alti (Hue, Saturation, Value)
    //ho bisogno di due intervalli per avere una maschera tra 330° e 30°
    Scalar lower_red2(int(0*179/360), 50, 10);
    Scalar upper_red2(int(5*179/360), 255, 255);
    // Crea una maschera che isola i pixel nell'intervallo
    Mat mask1, mask2;
    inRange(hsv, lower_red1, upper_red1, mask1);
    inRange(hsv, lower_red2, upper_red2, mask2);
    Mat mask;
    bitwise_or(mask1, mask2, mask);
    // Riduci il rumore applicando operazioni morfologiche
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(5, 5));
    erode(mask, mask, kernel);
    //dilate(mask, mask, kernel);

    // Trova i contorni nella maschera
    vector<vector<Point>> contours;
    double areaMinima = 1000.;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (!contours.empty()) {
        // Trova il contorno con l'area più grande (probabilmente la pallina)
        vector<Point> largest_contour;
        double max_area = 0;
        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > max_area && area > areaMinima) {
                max_area = area;
                largest_contour = contours[i];
            }
        }

        // disegna un cerchio nel centro dell'immagine
        circle(undistortedFrame, Point(goalY,goalX), 8, Scalar(0, 255, 0), -1);

        // Trova il centroide del contorno più grande
        Moments M = moments(largest_contour);
        if (M.m00 > 0) {
            cx = int(M.m01 / M.m00);
            cy = int(M.m10 / M.m00);

            // Disegna un cerchio sul centro della pallina
            circle(undistortedFrame, Point(cy, cx), 8, Scalar(255, 0, 0), -1);

            // Calcola il rettangolo di delimitazione attorno al contorno
            Rect bounding_box = boundingRect(largest_contour);

            // Disegna il rettangolo attorno alla pallina
            rectangle(undistortedFrame, bounding_box, Scalar(0, 255, 0), 2);  // Rettangolo verde
            //confermo di aver trovato la pallina
            ball_found = 1;
        }
    }
    // Converti la maschera binaria in un'immagine a colori (3 canali)
    Mat mask_colored;
    cvtColor(mask, mask_colored, COLOR_GRAY2BGR);  // Da scala di grigi a BGR

    // Creare l'overlay della maschera (ad esempio, maschera arancione trasparente)
    Mat overlay;
    undistortedFrame.copyTo(overlay);  // Copia il frame originale
    overlay.setTo(Scalar(0, 255, 255), mask);

    // Mescola l'immagine originale con l'overlay della maschera
    Mat blended;
    addWeighted(undistortedFrame, 0.7, overlay, 0.3, 0, blended);  // Unisce l'immagine originale con l'overlay

    // Mostra il risultato finale
    imshow("Webcam con Overlay", blended);
    int* out = new int[3];
    if(ball_found == 1) {
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
  //array con le 3 posizioni da restituire
  static int pos[3];
  //parametri del porblema
  double l = 80.; //lunghezza dei singoli bracci
  double b = 90.; //raggio di base
  double L = 117.7; //raggio del piatto
  //calcolo il valore di A,B,C
  double A = -(tan(thx) / sqrt(pow(1/cos(thy), 2) + pow(tan(thx), 2)));
  double B = -(tan(thy) / sqrt(pow(1 / cos(thy), 2) + pow(tan(thx), 2)));
  double C = -(1 / sqrt(pow(1 / cos(thy), 2) + pow(tan(thx), 2)));

  //----------------------------calcolo la posizione del primo motore-----------------------------------------------------------------------------
  //posizione del giunto sferico
  double x1 = abs((L * L * A * C) / sqrt(L * L * A * A * (A * A + C * C)));
  double y1 = 0;
  double z1 = h - (L * L * A * A) / sqrt(L * L * A * A * (A * A + C * C)) * abs(A * C) / (A * C);
  //angolo in radianti rispetto alla posizione orizzontale
  double th1 = acos((-b + (pow(b, 3) - pow(b, 2) * x1 - b * pow(x1, 2) + pow(x1, 3) - b * pow(y1, 2) + x1 * pow(y1, 2) + b * pow(z1, 2) + x1 * pow(z1, 2) +
              sqrt(-pow(z1, 2) * (pow(b, 4) - 4 * pow(b, 3) * x1 - 4 * pow(l, 2) * (pow(x1, 2) + pow(z1, 2)) + 
              pow(pow(x1, 2) + pow(y1, 2) + pow(z1, 2), 2) - 4 * b * x1 * (-2 * pow(l, 2) + pow(x1, 2) + pow(y1, 2) + pow(z1, 2)) + 
              2 * pow(b, 2) * (-2 * pow(l, 2) + 3 * pow(x1, 2) + pow(y1, 2) + pow(z1, 2))))) / 
              (2 * (pow(b, 2) - 2 * b * x1 + pow(x1, 2) + pow(z1, 2)))) / l);

  //----------------------------calcolo la posizione del secondo motore-----------------------------------------------------------------------------
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

  //----------------------------calcolo la posizione del terzo motore-----------------------------------------------------------------------------
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

  //salvo i risultati nelle variabili pos e li converto da radianti a numero di step
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

    //double kP=0.00035;
    //double kI=0.000025;
    //double kD=0.17;

    double th;
    //calcolo l'errore come differenza tra posizione e obbiettivo
    int err = pos-goal;

    //termine proporzionale
    double P = kP*err;
    
    //termine integrativo
    integral += err;
    double I = kI*integral;

    //termine derivativo
    double D = kD*(err-previous)/Dt;

    previous = err;
    //cout << P << " " << I << " " << D << endl;
    th = P + I + D;
    return th;
}

string receiveData(HANDLE hSerial)
{
    char buffer[256];
    DWORD bytesRead;
    ReadFile(hSerial, buffer, sizeof(buffer)-1, &bytesRead, NULL);
    return std::string(buffer, bytesRead);
}

void sendData(HANDLE hSerial, const string& data)
{
    DWORD bytesWritten;
    WriteFile(hSerial, data.c_str(), data.size(), &bytesWritten, NULL);
}

void limit(double &thx, double& thy, double& h)
{
    // limiting thx and thy to [-0.5,0.5]
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

void controlLoop(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos, auto tStart, int goalX, int goalY) {
    //tengo traccia del tempo attuale
        auto tEnd = steady_clock::now();
        auto time = duration_cast<milliseconds>(tEnd - tStart);
        if(Log) log_file << time.count() << ",";

        //step 1: calcolo la posizione della pallina
        int* ballPos = ballPosition(cap,ball_found,cx,cy);

        //posizione a riposo
        double thx = 0.0001;
        double thy = 0.0001;
        double h = 120;

        auto Dt = duration_cast<milliseconds>(tEnd - previousTime);
        int dt = static_cast<int>(Dt.count());
        if(ball_found==1) {
            //step 2: con il PID calcolo gli angoli thx e thy richiesti
            thx = PID(ballPos[0], goalX, integralX, prevX, dt);
            thy = -PID(ballPos[1], goalY, integralY, prevY, dt);
            h = 120;
            //per proteggere il sistema da divisioni per 0 mi assicuro che thx e thy non siano mai 0
            if(thx==0)
                thx=0.0001;
            if(thy==0)
                thy=0.0001;

            //limito la posizione dei motori
            limit(thx, thy, h);
        }

        //aggiorno il tempo precedente
        previousTime = tEnd;

        //step 3: calcolo la posizione che i motori devono raggiungere
        int* pos = motorPosition(thx, thy, h);

        //step 4: mando i dati ad arduino tramite porta seriale
        string to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
        sendData(hSerial, to_send);

        if(Log) log_file << ballPos[0] << "," << ballPos[1] << "," << thx << "," << thy << "," << h << "," << pos[0] << "," << pos[1] << "," << pos[2] << endl;
        //libero la memoria dinamica
        //delete[] pos;
        if(Log) log_real_pos << receiveData(hSerial);
}

/*
  ____                                          _     
 / ___|___  _ __ ___  _ __ ___   __ _ _ __   __| |___ 
| |   / _ \| '_ ` _ \| '_ ` _ \ / _` | '_ \ / _` / __|
| |__| (_) | | | | | | | | | | | (_| | | | | (_| \__ \
 \____\___/|_| |_| |_|_| |_| |_|\__,_|_| |_|\__,_|___/
*/

void jump(HANDLE hSerial)
{
    // jump sequence
    this_thread::sleep_for(seconds(2));
    int speed = 200;
    int* pos = motorPosition(0.0001, 0.0001, 70);
    string to_send;
    to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
    sendData(hSerial, to_send);
    cout << to_send << endl;
    this_thread::sleep_for(seconds(6));
    speed = 2000;
    pos = motorPosition(0.0001, 0.0001, 130);
    to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
    sendData(hSerial, to_send);
    cout << to_send << endl;
}

void balance(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto tStart = steady_clock::now();
    auto previousTime = tStart;
    //preparo il termine integrativo per il PID
    double integralX = 0; double integralY = 0;
    //salvo il valore della posizione precedente della pallina
    double prevX = 0; double prevY = 0;
    bool ball_found;
    //calcolo i parametri della webcam e il centro
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    int speed = 500;

    //ciclo per catturare ogni frame
    while(true) {
        controlLoop(hSerial,cap,log_file,log_real_pos,tStart,cx,cy);
        if(_kbhit())
            break;
        
        if(waitKey(1) >= 0) {
            break;
        }
    }
}

/*
void balance(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto tStart = steady_clock::now();
    auto previousTime = tStart;
    //preparo il termine integrativo per il PID
    double integralX = 0; double integralY = 0;
    //salvo il valore della posizione precedente della pallina
    double prevX = 0; double prevY = 0;
    bool ball_found;
    //calcolo i parametri della webcam e il centro
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    int speed = 500;

    //ciclo per catturare ogni frame
    while(true) {
        //tengo traccia del tempo attuale
        auto tEnd = steady_clock::now();
        auto time = duration_cast<milliseconds>(tEnd - tStart);
        if(Log) log_file << time.count() << ",";

        //step 1: calcolo la posizione della pallina
        int* ballPos = ballPosition(cap, ball_found,cx,cy);

        //posizione a riposo
        double thx = 0.0001;
        double thy = 0.0001;
        double h = 120;

        auto Dt = duration_cast<milliseconds>(tEnd - previousTime);
        int dt = static_cast<int>(Dt.count());
        if(ball_found==1) {
            //step 2: con il PID calcolo gli angoli thx e thy richiesti
            thx = PID(ballPos[0], cx, integralX, prevX, dt);
            thy = -PID(ballPos[1], cy, integralY, prevY, dt);
            h = 120;
            //per proteggere il sistema da divisioni per 0 mi assicuro che thx e thy non siano mai 0
            if(thx==0)
                thx=0.0001;
            if(thy==0)
                thy=0.0001;

            //limito la posizione dei motori
            limit(thx, thy, h);
        }

        //aggiorno il tempo precedente
        previousTime = tEnd;

        //step 3: calcolo la posizione che i motori devono raggiungere
        int* pos = motorPosition(thx, thy, h);

        //step 4: mando i dati ad arduino tramite porta seriale
        string to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
        sendData(hSerial, to_send);

        if(Log) log_file << ballPos[0] << "," << ballPos[1] << "," << thx << "," << thy << "," << h << "," << pos[0] << "," << pos[1] << "," << pos[2] << endl;
        //libero la memoria dinamica
        //delete[] pos;
        if(Log) log_real_pos << receiveData(hSerial);

        if(_kbhit())
            break;
        
        if(waitKey(1) >= 0) {
            break;
        }
    }
}
*/
void circle(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto tStart = steady_clock::now();
    auto previousTime = tStart;
    //preparo il termine integrativo per il PID
    double integralX = 0; double integralY = 0;
    //salvo il valore della posizione precedente della pallina
    double prevX = 0; double prevY = 0;
    bool ball_found;
    //calcolo i parametri della webcam e il centro
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    double r = 160;
    double theta = 0;
    int speed = 500;

    //ciclo per catturare ogni frame
    while(true) {
        if(theta < 2*PI)
            theta += 2*2*PI/180;
        else
            theta = 0;
        
        //tengo traccia del tempo attuale
        auto tEnd = steady_clock::now();
        auto time = duration_cast<milliseconds>(tEnd - tStart);
        if(Log) log_file << time.count() << ",";

        //step 1: calcolo la posizione della pallina
        int* ballPos = ballPosition(cap, ball_found,cx+r*sin(theta),cy+r*cos(theta));

        //posizione a riposo
        double thx = 0.0001;
        double thy = 0.0001;
        double h = 120;

        auto Dt = duration_cast<milliseconds>(tEnd - previousTime);
        int dt = static_cast<int>(Dt.count());

        if(ball_found==1) {
            //step 2: con il PID calcolo gli angoli thx e thy richiesti
            thx = PID(ballPos[0], cx+r*sin(theta), integralX, prevX, dt);
            thy = -PID(ballPos[1], cy+r*cos(theta), integralY, prevY, dt);
            h = 120;
            //per proteggere il sistema da divisioni per 0 mi assicuro che thx e thy non siano mai 0
            if(thx==0)
                thx=0.0001;
            if(thy==0)
                thy=0.0001;

            //limito la posizione dei motori
            limit(thx, thy, h);
        } 

        //aggiorno il tempo precedente
        previousTime = tEnd;

        //step 3: calcolo la posizione che i motori devono raggiungere
        int* pos = motorPosition(thx, thy, h);

        //step 4: mando i dati ad arduino tramite porta seriale
        string to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
        sendData(hSerial, to_send);

        if(Log) log_file << ballPos[0] << "," << ballPos[1] << "," << thx << "," << thy << "," << h << "," << pos[0] << "," << pos[1] << "," << pos[2] << endl;
        //libero la memoria dinamica
        //delete[] pos;
        if(Log) log_real_pos << receiveData(hSerial);

        if(_kbhit())
            break;

        if(waitKey(1) >= 0)
            break;
    }
}

void star(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto tStart = steady_clock::now();
    auto previousTime = tStart;
    //preparo il termine integrativo per il PID
    double integralX = 0; double integralY = 0;
    //salvo il valore della posizione precedente della pallina
    double prevX = 0; double prevY = 0;
    bool ball_found;
    //calcolo i parametri della webcam e il centro
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    double r = 0;
    double Rmin = 80;
    double Rmax = 160;
    int n = 5;
    int p = 2;
    double theta = 0;
    int speed = 500;

    //ciclo per catturare ogni frame
    while(true) {
        if(theta < 2*PI)
            theta += 1*2*PI/180;
        else
            theta = 0;

        r = Rmin + (Rmax-Rmin)*pow((1+cos(n*theta))/2,p);
        
        //tengo traccia del tempo attuale
        auto tEnd = steady_clock::now();
        auto time = duration_cast<milliseconds>(tEnd - tStart);
        if(Log) log_file << time.count() << ",";

        //step 1: calcolo la posizione della pallina
        int* ballPos = ballPosition(cap, ball_found,cx+r*sin(theta),cy+r*cos(theta));

        //posizione a riposo
        double thx = 0.0001;
        double thy = 0.0001;
        double h = 120;

        auto Dt = duration_cast<milliseconds>(tEnd - previousTime);
        int dt = static_cast<int>(Dt.count());

        if(ball_found==1) {
            //step 2: con il PID calcolo gli angoli thx e thy richiesti
            thx = PID(ballPos[0], cx+r*sin(theta), integralX, prevX, dt);
            thy = -PID(ballPos[1], cy+r*cos(theta), integralY, prevY, dt);
            h = 120;
            //per proteggere il sistema da divisioni per 0 mi assicuro che thx e thy non siano mai 0
            if(thx==0)
                thx=0.0001;
            if(thy==0)
                thy=0.0001;

            //limito la posizione dei motori
            limit(thx, thy, h);
        } 

        //aggiorno il tempo precedente
        previousTime = tEnd;

        //step 3: calcolo la posizione che i motori devono raggiungere
        int* pos = motorPosition(thx, thy, h);

        //step 4: mando i dati ad arduino tramite porta seriale
        string to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
        sendData(hSerial, to_send);

        if(Log) log_file << ballPos[0] << "," << ballPos[1] << "," << thx << "," << thy << "," << h << "," << pos[0] << "," << pos[1] << "," << pos[2] << endl;
        //libero la memoria dinamica
        //delete[] pos;
        if(Log) log_real_pos << receiveData(hSerial);

        if(_kbhit())
            break;

        if(waitKey(1) >= 0)
            break;
    }
}

void square(HANDLE hSerial, VideoCapture cap, ofstream& log_file, ofstream& log_real_pos) {
    auto tStart = steady_clock::now();
    auto previousTime = tStart;
    //preparo il termine integrativo per il PID
    double integralX = 0; double integralY = 0;
    //salvo il valore della posizione precedente della pallina
    double prevX = 0; double prevY = 0;
    bool ball_found;
    //calcolo i parametri della webcam e il centro
    int cx = int(cap.get(CAP_PROP_FRAME_HEIGHT)/2);
    int cy = int(cap.get(CAP_PROP_FRAME_WIDTH)/2);
    double r = 0;
    double a = 160;
    double theta = 0;
    int speed = 500;

    //ciclo per catturare ogni frame
    while(true) {
        if(theta < 2*PI)
            theta += 1*2*PI/180;
        else
            theta = 0;

        r = a/(max(abs(cos(theta)),abs(sin(theta))));
        
        //tengo traccia del tempo attuale
        auto tEnd = steady_clock::now();
        auto time = duration_cast<milliseconds>(tEnd - tStart);
        if(Log) log_file << time.count() << ",";

        //step 1: calcolo la posizione della pallina
        int* ballPos = ballPosition(cap, ball_found,cx+r*sin(theta),cy+r*cos(theta));

        //posizione a riposo
        double thx = 0.0001;
        double thy = 0.0001;
        double h = 120;

        auto Dt = duration_cast<milliseconds>(tEnd - previousTime);
        int dt = static_cast<int>(Dt.count());

        if(ball_found==1) {
            //step 2: con il PID calcolo gli angoli thx e thy richiesti
            thx = PID(ballPos[0], cx+r*sin(theta), integralX, prevX, dt);
            thy = -PID(ballPos[1], cy+r*cos(theta), integralY, prevY, dt);
            h = 120;
            //per proteggere il sistema da divisioni per 0 mi assicuro che thx e thy non siano mai 0
            if(thx==0)
                thx=0.0001;
            if(thy==0)
                thy=0.0001;

            //limito la posizione dei motori
            limit(thx, thy, h);
        } 

        //aggiorno il tempo precedente
        previousTime = tEnd;

        //step 3: calcolo la posizione che i motori devono raggiungere
        int* pos = motorPosition(thx, thy, h);

        //step 4: mando i dati ad arduino tramite porta seriale
        string to_send = to_string(pos[0]) + '_' + to_string(pos[1]) + '_' + to_string(pos[2]) + '_' + to_string(speed) + '\n';
        sendData(hSerial, to_send);

        if(Log) log_file << ballPos[0] << "," << ballPos[1] << "," << thx << "," << thy << "," << h << "," << pos[0] << "," << pos[1] << "," << pos[2] << endl;
        //libero la memoria dinamica
        //delete[] pos;
        if(Log) log_real_pos << receiveData(hSerial);

        if(_kbhit())
            break;

        if(waitKey(1) >= 0)
            break;
    }
}


/*
 __  __       _       
|  \/  | __ _(_)_ __  
| |\/| |/ _` | | '_ \ 
| |  | | (_| | | | | |
|_|  |_|\__,_|_|_| |_|
*/

int main()
{
    HANDLE hSerial;
    VideoCapture cap(0);
    ofstream log_file("log.csv");
    ofstream log_real_pos("log_actual.csv");
    connectToRobot(hSerial,cap,log_file,log_real_pos);

    while(true) {
        cout << "Comandi disponibili: " <<  "b " << "per bilanciare la pallina, " << "s " << "per saltare, " <<
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
    //chiudo la webcam
    cap.release();
    destroyWindow("Webcam con Overlay");
    //Chiudo la porta seriale
    CloseHandle(hSerial);
    log_file.close();
    return 0;
}




//per compilare:
//  mingw32-make
//  ./ProvaOpenCV.exe



//Camera calibration
/*
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

using namespace cv;
using namespace std;

static void help()
{
    cout <<  "This is a camera calibration sample." << endl
         <<  "Usage: calibration configurationFile"  << endl
         <<  "Near the sample file you'll find the configuration file, which has detailed help of "
             "how to edit it.  It may be any OpenCV supported file format XML/YAML." << endl;
}
class Settings
{
public:
    Settings() : goodInput(false) {}
    enum Pattern { NOT_EXISTING, CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
    enum InputType {INVALID, CAMERA, VIDEO_FILE, IMAGE_LIST};

    void write(FileStorage& fs) const                        //Write serialization for this class
    {
        fs << "{" << "BoardSize_Width"  << boardSize.width
                  << "BoardSize_Height" << boardSize.height
                  << "Square_Size"         << squareSize
                  << "Calibrate_Pattern" << patternToUse
                  << "Calibrate_NrOfFrameToUse" << nrFrames
                  << "Calibrate_FixAspectRatio" << aspectRatio
                  << "Calibrate_AssumeZeroTangentialDistortion" << calibZeroTangentDist
                  << "Calibrate_FixPrincipalPointAtTheCenter" << calibFixPrincipalPoint

                  << "Write_DetectedFeaturePoints" << bwritePoints
                  << "Write_extrinsicParameters"   << bwriteExtrinsics
                  << "Write_outputFileName"  << outputFileName

                  << "Show_UndistortedImage" << showUndistorsed

                  << "Input_FlipAroundHorizontalAxis" << flipVertical
                  << "Input_Delay" << delay
                  << "Input" << input
           << "}";
    }
    void read(const FileNode& node)                          //Read serialization for this class
    {
        node["BoardSize_Width" ] >> boardSize.width;
        node["BoardSize_Height"] >> boardSize.height;
        node["Calibrate_Pattern"] >> patternToUse;
        node["Square_Size"]  >> squareSize;
        node["Calibrate_NrOfFrameToUse"] >> nrFrames;
        node["Calibrate_FixAspectRatio"] >> aspectRatio;
        node["Write_DetectedFeaturePoints"] >> bwritePoints;
        node["Write_extrinsicParameters"] >> bwriteExtrinsics;
        node["Write_outputFileName"] >> outputFileName;
        node["Calibrate_AssumeZeroTangentialDistortion"] >> calibZeroTangentDist;
        node["Calibrate_FixPrincipalPointAtTheCenter"] >> calibFixPrincipalPoint;
        node["Input_FlipAroundHorizontalAxis"] >> flipVertical;
        node["Show_UndistortedImage"] >> showUndistorsed;
        node["Input"] >> input;
        node["Input_Delay"] >> delay;
        interprate();
    }
    void interprate()
    {
        goodInput = true;
        if (boardSize.width <= 0 || boardSize.height <= 0)
        {
            cerr << "Invalid Board size: " << boardSize.width << " " << boardSize.height << endl;
            goodInput = false;
        }
        if (squareSize <= 10e-6)
        {
            cerr << "Invalid square size " << squareSize << endl;
            goodInput = false;
        }
        if (nrFrames <= 0)
        {
            cerr << "Invalid number of frames " << nrFrames << endl;
            goodInput = false;
        }

        if (input.empty())      // Check for valid input
                inputType = INVALID;
        else
        {
            if (input[0] >= '0' && input[0] <= '9')
            {
                stringstream ss(input);
                ss >> cameraID;
                inputType = CAMERA;
            }
            else
            {
                if (readStringList(input, imageList))
                    {
                        inputType = IMAGE_LIST;
                        nrFrames = (nrFrames < (int)imageList.size()) ? nrFrames : (int)imageList.size();
                    }
                else
                    inputType = VIDEO_FILE;
            }
            if (inputType == CAMERA)
                inputCapture.open(cameraID);
            if (inputType == VIDEO_FILE)
                inputCapture.open(input);
            if (inputType != IMAGE_LIST && !inputCapture.isOpened())
                    inputType = INVALID;
        }
        if (inputType == INVALID)
        {
            cerr << " Inexistent input: " << input;
            goodInput = false;
        }

        flag = 0;
        if(calibFixPrincipalPoint) flag |= CALIB_FIX_PRINCIPAL_POINT;
        if(calibZeroTangentDist)   flag |= CALIB_ZERO_TANGENT_DIST;
        if(aspectRatio)            flag |= CALIB_FIX_ASPECT_RATIO;


        calibrationPattern = NOT_EXISTING;
        if (!patternToUse.compare("CHESSBOARD")) calibrationPattern = CHESSBOARD;
        if (!patternToUse.compare("CIRCLES_GRID")) calibrationPattern = CIRCLES_GRID;
        if (!patternToUse.compare("ASYMMETRIC_CIRCLES_GRID")) calibrationPattern = ASYMMETRIC_CIRCLES_GRID;
        if (calibrationPattern == NOT_EXISTING)
            {
                cerr << " Inexistent camera calibration mode: " << patternToUse << endl;
                goodInput = false;
            }
        atImageList = 0;

    }
    Mat nextImage()
    {
        Mat result;
        if( inputCapture.isOpened() )
        {
            Mat view0;
            inputCapture >> view0;
            view0.copyTo(result);
        }
        else if( atImageList < (int)imageList.size() )
            result = imread(imageList[atImageList++], IMREAD_COLOR);

        return result;
    }

    static bool readStringList( const string& filename, vector<string>& l )
    {
        l.clear();
        FileStorage fs(filename, FileStorage::READ);
        if( !fs.isOpened() )
            return false;
        FileNode n = fs.getFirstTopLevelNode();
        if( n.type() != FileNode::SEQ )
            return false;
        FileNodeIterator it = n.begin(), it_end = n.end();
        for( ; it != it_end; ++it )
            l.push_back((string)*it);
        return true;
    }
public:
    Size boardSize;            // The size of the board -> Number of items by width and height
    Pattern calibrationPattern;// One of the Chessboard, circles, or asymmetric circle pattern
    float squareSize;          // The size of a square in your defined unit (point, millimeter,etc).
    int nrFrames;              // The number of frames to use from the input for calibration
    float aspectRatio;         // The aspect ratio
    int delay;                 // In case of a video input
    bool bwritePoints;         //  Write detected feature points
    bool bwriteExtrinsics;     // Write extrinsic parameters
    bool calibZeroTangentDist; // Assume zero tangential distortion
    bool calibFixPrincipalPoint;// Fix the principal point at the center
    bool flipVertical;          // Flip the captured images around the horizontal axis
    string outputFileName;      // The name of the file where to write
    bool showUndistorsed;       // Show undistorted images after calibration
    string input;               // The input ->



    int cameraID;
    vector<string> imageList;
    int atImageList;
    VideoCapture inputCapture;
    InputType inputType;
    bool goodInput;
    int flag;

private:
    string patternToUse;


};

static void read(const FileNode& node, Settings& x, const Settings& default_value = Settings())
{
    if(node.empty())
        x = default_value;
    else
        x.read(node);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,
                           vector<vector<Point2f> > imagePoints );

int main(int argc, char* argv[])
{
    help();
    Settings s;
    const string inputSettingsFile = argc > 1 ? argv[1] : "configs/default.xml";
    FileStorage fs(inputSettingsFile, FileStorage::READ); // Read the settings
    if (!fs.isOpened())
    {
        cout << "Could not open the configuration file: \"" << inputSettingsFile << "\"" << endl;
        return -1;
    }
    fs["Settings"] >> s;
    fs.release();                                         // close Settings file

    if (!s.goodInput)
    {
        cout << "Invalid input detected. Application stopping. " << endl;
        return -1;
    }

    vector<vector<Point2f> > imagePoints;
    Mat cameraMatrix, distCoeffs;
    Size imageSize;
    int mode = s.inputType == Settings::IMAGE_LIST ? CAPTURING : DETECTION;
    clock_t prevTimestamp = 0;
    const Scalar RED(0,0,255), GREEN(0,255,0);
    const char ESC_KEY = 27;

    for(int i = 0;;++i)
    {
      Mat view;
      bool blinkOutput = false;

      view = s.nextImage();

      //-----  If no more image, or got enough, then stop calibration and show result -------------
      if( mode == CAPTURING && imagePoints.size() >= (unsigned)s.nrFrames )
      {
          if( runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints))
              mode = CALIBRATED;
          else
              mode = DETECTION;
      }
      if(view.empty())          // If no more images then run calibration, save and stop loop.
      {
            if( imagePoints.size() > 0 )
                runCalibrationAndSave(s, imageSize,  cameraMatrix, distCoeffs, imagePoints);
            break;
      }


        imageSize = view.size();  // Format input image.
        if( s.flipVertical )    flip( view, view, 0 );

        vector<Point2f> pointBuf;

        bool found;
        switch( s.calibrationPattern ) // Find feature points on the input format
        {
        case Settings::CHESSBOARD:
            found = findChessboardCorners( view, s.boardSize, pointBuf,
                CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
            break;
        case Settings::CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf );
            break;
        case Settings::ASYMMETRIC_CIRCLES_GRID:
            found = findCirclesGrid( view, s.boardSize, pointBuf, CALIB_CB_ASYMMETRIC_GRID );
            break;
        default:
            found = false;
            break;
        }

        if ( found)                // If done with success,
        {
              // improve the found corners' coordinate accuracy for chessboard
                if( s.calibrationPattern == Settings::CHESSBOARD)
                {
                    Mat viewGray;
                    cvtColor(view, viewGray, COLOR_BGR2GRAY);
                    cornerSubPix( viewGray, pointBuf, Size(11,11),
                        Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::MAX_ITER, 30, 0.1 ));
                }

                if( mode == CAPTURING &&  // For camera only take new samples after delay time
                    (!s.inputCapture.isOpened() || clock() - prevTimestamp > s.delay*1e-3*CLOCKS_PER_SEC) )
                {
                    imagePoints.push_back(pointBuf);
                    prevTimestamp = clock();
                    blinkOutput = s.inputCapture.isOpened();
                }

                // Draw the corners.
                drawChessboardCorners( view, s.boardSize, Mat(pointBuf), found );
        }

        //----------------------------- Output Text ------------------------------------------------
        string msg = (mode == CAPTURING) ? "100/100" :
                      mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(s.showUndistorsed)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), s.nrFrames );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), s.nrFrames );
        }

        putText( view, msg, textOrigin, 1, 1, mode == CALIBRATED ?  GREEN : RED);

        if( blinkOutput )
            bitwise_not(view, view);

        //------------------------- Video capture  output  undistorted ------------------------------
        if( mode == CALIBRATED && s.showUndistorsed )
        {
            Mat temp = view.clone();
            undistort(temp, view, cameraMatrix, distCoeffs);
        }

        //------------------------------ Show image and check for input commands -------------------
        imshow("Image View", view);
        char key = (char)waitKey(s.inputCapture.isOpened() ? 50 : s.delay);

        if( key  == ESC_KEY )
            break;

        if( key == 'u' && mode == CALIBRATED )
           s.showUndistorsed = !s.showUndistorsed;

        if( s.inputCapture.isOpened() && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }
    }

    // -----------------------Show the undistorted image for the image list ------------------------
    if( s.inputType == Settings::IMAGE_LIST && s.showUndistorsed )
    {
        Mat view, rview, map1, map2;
        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
            getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
            imageSize, CV_16SC2, map1, map2);

        for(int i = 0; i < (int)s.imageList.size(); i++ )
        {
            view = imread(s.imageList[i], 1);
            if(view.empty())
                continue;
            remap(view, rview, map1, map2, INTER_LINEAR);
            imshow("Image View", rview);
            char c = (char)waitKey();
            if( c  == ESC_KEY || c == 'q' || c == 'Q' )
                break;
        }
    }


    return 0;
}

static double computeReprojectionErrors( const vector<vector<Point3f> >& objectPoints,
                                         const vector<vector<Point2f> >& imagePoints,
                                         const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                         const Mat& cameraMatrix , const Mat& distCoeffs,
                                         vector<float>& perViewErrors)
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners,
                                     Settings::Pattern patternType)
{
    corners.clear();

    switch(patternType)
    {
    case Settings::CHESSBOARD:
    case Settings::CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; ++i )
            for( int j = 0; j < boardSize.width; ++j )
                corners.push_back(Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        break;

    case Settings::ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize), float(i*squareSize), 0));
        break;
    default:
        break;
    }
}

static bool runCalibration( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                            vector<vector<Point2f> > imagePoints, vector<Mat>& rvecs, vector<Mat>& tvecs,
                            vector<float>& reprojErrs,  double& totalAvgErr)
{

    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = 1.0;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcBoardCornerPositions(s.boardSize, s.squareSize, objectPoints[0], s.calibrationPattern);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    //Find intrinsic and extrinsic camera parameters
    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                                 distCoeffs, rvecs, tvecs, s.flag|CALIB_FIX_K4|CALIB_FIX_K5);

    cout << "Re-projection error reported by calibrateCamera: "<< rms << endl;

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}

// Print camera parameters to the output file
static void saveCameraParams( Settings& s, Size& imageSize, Mat& cameraMatrix, Mat& distCoeffs,
                              const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                              const vector<float>& reprojErrs, const vector<vector<Point2f> >& imagePoints,
                              double totalAvgErr )
{
    FileStorage fs( s.outputFileName, FileStorage::WRITE );

    time_t tm;
    time( &tm );
    struct tm *t2 = localtime( &tm );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_Time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nrOfFrames" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_Width" << imageSize.width;
    fs << "image_Height" << imageSize.height;
    fs << "board_Width" << s.boardSize.width;
    fs << "board_Height" << s.boardSize.height;
    fs << "square_Size" << s.squareSize;

    if( s.flag & CALIB_FIX_ASPECT_RATIO )
        fs << "FixAspectRatio" << s.aspectRatio;

    if( s.flag )
    {
        sprintf( buf, "flags: %s%s%s%s",
            s.flag & CALIB_USE_INTRINSIC_GUESS ? " +use_intrinsic_guess" : "",
            s.flag & CALIB_FIX_ASPECT_RATIO ? " +fix_aspectRatio" : "",
            s.flag & CALIB_FIX_PRINCIPAL_POINT ? " +fix_principal_point" : "",
            s.flag & CALIB_ZERO_TANGENT_DIST ? " +zero_tangent_dist" : "" );
        //cvWriteComment( *fs, buf, 0 );

    }

    fs << "flagValue" << s.flag;

    fs << "Camera_Matrix" << cameraMatrix;
    fs << "Distortion_Coefficients" << distCoeffs;

    fs << "Avg_Reprojection_Error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "Per_View_Reprojection_Errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "Extrinsic_Parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "Image_points" << imagePtMat;
    }
}

bool runCalibrationAndSave(Settings& s, Size imageSize, Mat&  cameraMatrix, Mat& distCoeffs,vector<vector<Point2f> > imagePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(s,imageSize, cameraMatrix, distCoeffs, imagePoints, rvecs, tvecs,
                             reprojErrs, totalAvgErr);
    cout << (ok ? "Calibration succeeded" : "Calibration failed")
        << ". avg re projection error = "  << totalAvgErr ;

    if( ok )
        saveCameraParams( s, imageSize, cameraMatrix, distCoeffs, rvecs ,tvecs, reprojErrs,
                            imagePoints, totalAvgErr);
    return ok;
}
*/