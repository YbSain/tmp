#include <opencv2/opencv.hpp>
#include <iostream>
#include <signal.h>

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
int main(void) {
    TickMeter tm;
    VideoCapture cap("7_lt_ccw_100rpm_in.mp4");

    if (!cap.isOpened()) { cerr << "Video Open Failed!" << endl; return -1; }

    Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }
        Mat Threshframe = makethresh(frame);
        Mat tmp = Threshframe.clone();
        Mat displaymorph = regray(tmp);
        Labeling(Threshframe, displaymorph);

        imshow("morph", displaymorph);
        imshow("cap", frame);
        if (waitKey(30) == 27) break;
    }
    cap.release();
    destroyAllWindows();
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

    Scalar meanValueAdjust = mean(Roiframe);
    double adjustedBrightness = meanValueAdjust[0];
    //이진화
    threshold(Roiframe, Roiframe, 165, 255, THRESH_BINARY);
    //라인 검출

    //모폴로지 / 노이즈
    morphologyEx(Roiframe, Roiframe, MORPH_OPEN, Mat(), Point(-1, -1));

    return Roiframe;
}
Mat regray(Mat Threshframe) {
    erode(Threshframe, Threshframe, Mat(), Point(-1, -1), 2);
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
        int error = Threshframe.cols / 2 - (int)c[0];
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
                cout << "error" << error << endl;
            }
        }
    }
}
