#include <opencv2/opencv.hpp>
#include <iostream>
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"

using namespace cv;
using namespace std;

bool ctrl_c_pressed;
void ctrlc(int) {
    ctrl_c_pressed = true;
}
Mat makethresh(Mat frame);
void Labeling(Mat displaymorph, Mat Grayframe);
Mat regray(Mat Threshframe);
Rect targetBoundingBox;
Point2d targetCenter;
bool isTarget = false;      //좌표확인
Dxl mx;
int getError(Mat& thresh, Point& tmp_pt);
struct timeval start, end1;
double diff;



bool mode = false;

int main(void) {
    TickMeter tm;
    //원본
    // string src = "nvarguscamerasrc sensor-id=0 ! 
    //     video/x-raw(memory:NVMM), width=(int)640, height=(int)360, 
    //     format=(string)NV12, framerate=(fraction)30/1 ! 
    //     nvvidconv flip-method=0 ! video/x-raw, 
    //     width=(int)640, height=(int)360, format=(string)BGRx ! 
    //     videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    VideoCapture cap("7_lt_ccw_100rpm_in.mp4");
    if (!cap.isOpened()) { cerr << "video open failed!" << endl; return -1; }

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
            nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
            h264parse ! rtph264pay pt=96 ! \
            udpsink host=203.234.58.14 port=8001 sync=false";
            
    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
            nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
            h264parse ! rtph264pay pt=96 ! \
            udpsink host=203.234.58.14 port=8002 sync=false";
    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }
    VideoWriter writer2(dst2, 0, (double)30, Size(640, 360), false);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

    Mat frame;
    int lval = 0, rval = 0;

    signal(SIGINT, ctrlc);


    while (true) {
        gettimeofday(&start, NULL);
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        Mat Threshframe = makethresh(frame);    //밝기 처리 + 이진화
        Mat tmp = Threshframe.clone();
        Mat displaymorph = regray(tmp);         //밝은 그림으로보기

        Point tmp_pt =targetCenter;
        if (ctrl_c_pressed) break;
        Labeling(Threshframe, displaymorph);


        usleep(30 * 1000);
        gettimeofday(&end1, NULL);
        int error = getError(Threshframe, tmp_pt);

        lval = 100 - 0.1 * error;
        rval = -(100 + 0.1 * error);
        if (!mx.open()) {
            cout << "Dxl error" << endl;
            return -1;
        }
        if (mx.kbhit()) {
            char ch = mx.getch();
            if (ch == 'q')break;
            else if (ch == 's') mode = true;
        }
        diff = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
        cout << "lval : " << lval << "\trval : " << rval << "\ttime : " << diff << endl;
        cout << "error: " << error;
        if (mode) mx.setVelocity(lval, rval);        
        
        writer1 << frame;
        writer2 << displaymorph;
    }
    mx.close();
    return 0;
}

Mat makethresh(Mat frame) {
    //전처리
        //Roi범위구하기
    int x = 0, y = 270, width = frame.cols, height = 90;
    Rect Roi(x, y, width, height);
    if (x + width > frame.cols || y + height > frame.rows) {
        cerr << "over frame" << endl;
    }
    Mat Roiframe = frame(Roi);

    cvtColor(Roiframe, Roiframe, COLOR_BGR2GRAY);
    Scalar meanValue = mean(Roiframe);
    double currentBrightness = meanValue[0];
    double targetBrightness = 100.0;
    double adjustment = (targetBrightness - currentBrightness);
    Roiframe.convertTo(Roiframe, -1, 1, adjustment);

    threshold(Roiframe, Roiframe, 165, 255, THRESH_BINARY);

    //모폴로지 / 노이즈
    morphologyEx(Roiframe, Roiframe, MORPH_OPEN, Mat(), Point(-1, -1));

    return Roiframe;
}
Mat regray(Mat Threshframe) {
    Mat displaymorph;
    cvtColor(Threshframe, displaymorph, COLOR_GRAY2BGR);
    return displaymorph;
}
void Labeling(Mat Threshframe, Mat Grayframe) {
    Mat labels, stats, centroids;
    static Point2d firstCenter(Threshframe.cols / 2, Threshframe.rows / 2);
    int cnt = connectedComponentsWithStats(Threshframe, labels, stats, centroids);
    for (int i = 1; i < cnt; i++) {
        int* p = stats.ptr<int>(i);
        double* c = centroids.ptr<double>(i);
        if (p[4] < 20) continue;
        Rect currentBoundingBox(p[0], p[1], p[2], p[3]);
        Point2d currentCenter(c[0], c[1]);
        
        rectangle(Grayframe, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 255, 255), 2);
        double xdistance = targetCenter.x - c[0];
        if (!isTarget) {
            targetCenter = firstCenter;
            isTarget = true;
        }
        else {
            if ((xdistance <= 80 && xdistance >= -80)) {
                rectangle(Grayframe, Rect(p[0], p[1], p[2], p[3]), Scalar(0, 0, 255), 2);
                circle(Grayframe, targetCenter, 3, Scalar(0, 255, 0), 2);
                circle(Grayframe, currentCenter, 5, Scalar(0, 255, 0), FILLED);
                targetBoundingBox = currentBoundingBox;
                targetCenter = currentCenter;
            }
        }
    }
}
int getError(Mat& Threshframe, Point& tmp_pt) {
    return ((Threshframe.cols / 2) - tmp_pt.x);
}
