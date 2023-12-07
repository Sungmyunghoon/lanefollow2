#include <iostream>
#include <signal.h>
#include <vector>
#include <algorithm>
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include <sys/time.h>
#include "dxl.hpp"
using namespace std;
using namespace cv;

bool ctrl_c_pressed = false;
void ctrlc_handler(int) { ctrl_c_pressed = true; }

int main() {
    Dxl mx;
    int vel1 = 0, vel2 = 0;
    signal(SIGINT, ctrlc_handler);
    if (!mx.open()) { cout << "dynamixel open error" << endl; return -1; }

    string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";

    //string src = "lanefollow_100rpm_ccw.mp4";

    VideoCapture cap(src,CAP_GSTREAMER);

    string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.168 port=8001 sync=false";

    VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);

    string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.168 port=8002 sync=false";

    VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true);

    Mat frame, black, dst, result_img;                                                // Mat 객체 생성
    Mat labels, stats, centroids;                                          // connectedComponentsWithStats을 생성하기 위해 필요한 객체
    //int vel1 = 0, vel2 = 0;
    int error = 0;
    bool mode = false;
    char stop;

    vector<double> vec_L;
    vector<double> vec_R;
    double distance_L = 0, distance_R = 0;

    while (true)
    {
        double start = getTickCount();                                              // 반복문 시간 시작단계
        cap >> frame;                                                                 // cap의 영상을 frame에 삽입
        cvtColor(frame, black, COLOR_BGR2GRAY);                                       // 이미지 그레이 컬러로 변경
        dst = black(Rect(0, 270, 640, 90));                                         // 크기 조절한 로드 맵

        double desiredMean = 90;                                                    // 밝기 보정
        Scalar meanValue = mean(dst);                                               // 밝기 보정 함수 mean사용
        double b = desiredMean - meanValue.val[0];                                  // 밝기 평균을 계산해서 b에 넣기
        result_img = dst + b;                                                       // 밝기 계산해서 원본 파일에 넣기

        threshold(result_img, result_img, 128, 255, THRESH_BINARY);                 // 이진화 128이상이면 255로 변경

        int p_width = result_img.cols;                                              // 이미지의 너비
        int p_height = result_img.rows;                                             // 이미지의 높이

        int numLabels = connectedComponentsWithStats(result_img, labels, stats, centroids);// 레이블링
        cvtColor(result_img, result_img, COLOR_GRAY2BGR);                       // 이미지 컬러로 변경

        static Point Left_pos((p_width) / 4, p_height / 2);                              // 주 로드를 잡을 기준점 변경되어야 함으로 static을 사용
        static Point Right_pos((p_width) / 4 * 3, p_height / 2);

        Point LeftInteger;
        Point RightInteger;

        if (mx.kbhit()) {
            stop = mx.getch();
            if (stop == 'q') break;
            else if (stop == 's') mode = true;
        }
        for (int i = 1; i < numLabels; i++) {                                       // 각각의 좌표점과 박스 색깔
            double x = centroids.at<double>(i, 0);                                  // 객체의 중심점 X
            double y = centroids.at<double>(i, 1);                                  // 객체의 중심점 Y

            LeftInteger = Left_pos - Point(x, y);                                       // 주 로드의 기준점과 객체의 무게중심 원점의 점
            RightInteger = Right_pos - Point(x, y);

            int label = (stats.at<int>(i, CC_STAT_AREA));                         // 레이블 기준으로 객체 구분
            if (label < 100)
            {
                vec_L.push_back(9999);
                vec_R.push_back(9999);
                continue;
            }

            distance_L = norm(Left_pos - Point(x, y));
            distance_R = norm(Right_pos - Point(x, y));

            vec_L.push_back(distance_L);
            vec_R.push_back(distance_R);
        }

        double min_L = *std::min_element(vec_L.begin(), vec_L.end());
        double min_R = *std::min_element(vec_R.begin(), vec_R.end());
        auto it_L = std::find(vec_L.begin(), vec_L.end(), min_L);
        auto it_R = std::find(vec_R.begin(), vec_R.end(), min_R);

        for (int i = 1; i < numLabels; i++)
        {
            start = getTickCount(); 
            int left = stats.at<int>(i, CC_STAT_LEFT);
            int top = stats.at<int>(i, CC_STAT_TOP);
            int width = stats.at<int>(i, CC_STAT_WIDTH);
            int height = stats.at<int>(i, CC_STAT_HEIGHT);

            double x = centroids.at<double>(i, 0);
            double y = centroids.at<double>(i, 1);

            int label = (stats.at<int>(i, CC_STAT_AREA));
            if (label < 100) continue;

            if ((i - 1 == distance(vec_L.begin(), it_L)) && (min_L < 80))
            {
                Left_pos = Point(x, y);                                                  // 객체의 점 이동
                rectangle(result_img, Point(left, top), Point(left + width, top + height), Scalar(0, 255, 0), 2); // 사각형
                circle(result_img, Point(static_cast<int>(x), static_cast<int>(y)), 3, Scalar(0, 0, 255), -1); // 원 (중심 좌표에 점 찍기)
            }
            if ((i - 1 == distance(vec_R.begin(), it_R)) && (min_R < 80))
            {
                Right_pos = Point(x, y);                                                  // 객체의 점 이동
                rectangle(result_img, Point(left, top), Point(left + width, top + height), Scalar(0, 255, 0), 2); // 사각형
                circle(result_img, Point(static_cast<int>(x), static_cast<int>(y)), 3, Scalar(0, 0, 255), -1); // 원 (중심 좌표에 점 찍기)
            }
            line(result_img, Left_pos, Right_pos, Scalar(0, 0, 255), 2);
            circle(result_img, Point(Left_pos.x + ((Right_pos.x - Left_pos.x) / 2), Left_pos.y + ((Right_pos.y - Left_pos.y) / 2)), 3, Scalar(0, 255, 0), -1);
            error = 320 - (Left_pos.x + ((Right_pos.x - Left_pos.x) / 2));
        }
        vec_L.clear();
        vec_R.clear();

        writer1 << frame;
        writer2 << result_img;

        vel1 = 200 - 0.4 * error;
        vel2 = -(200 + 0.4 * error);

        if (ctrl_c_pressed) break;
        if (mode) mx.setVelocity(vel1, vel2);                             // 기본 영상과 로드 영상
        usleep(20*1000);
        double end = getTickCount();                                                // 종료 시간 기록
        double elapsedTime = (end - start) / getTickFrequency();                    // 실행 시간 계산 (초 단위로 변환)
        cout << "error: " << error << " 실행 시간 : " << elapsedTime << " 초" << endl;// 결과 출력
    }
    mx.close();
    return 0;                                                                       // 반환
}