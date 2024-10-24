#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <vector>
#include <math.h>

using namespace cv;
using namespace std;

int main()
{
    VideoCapture cap("../cam.avi");

    if (!cap.isOpened())
    {
        cout << "Unable to access camera!" << endl;
        return -1;
    }

    Mat frame, frame_hsv, frame_thres;

    /**
     * @brief Configure Trackbar window
     *
     */
    int h[2] = {100, 255}, s[2] = {170, 255}, v[2] = {200, 255};
    namedWindow("Control");

    createTrackbar("Hmin", "Control", &h[0], 255);
    createTrackbar("Smin", "Control", &s[0], 255);
    createTrackbar("Vmin", "Control", &v[0], 255);

    createTrackbar("Hmax", "Control", &h[1], 255);
    createTrackbar("Smax", "Control", &s[1], 255);
    createTrackbar("Vmax", "Control", &v[1], 255);

    Point2f startBallPosition(0, 0);

    while (true)
    {
        cap >> frame;

        // Convert RGB to HSV
        cvtColor(frame, frame_hsv, COLOR_RGB2HSV);
        inRange(frame_hsv, Scalar(h[0], s[0], v[0]), Scalar(h[1], s[1], v[1]), frame_thres);

        vector<vector<Point>> contours;
        findContours(frame_thres, contours, RETR_TREE, CHAIN_APPROX_NONE);

        if (!contours.empty())
        {
            // Get center of robot
            Point2f centerOfRobot(frame.cols / 2, (frame.rows / 2) - 20);
            circle(frame, centerOfRobot, 4, Scalar(0, 0, 255), -1);

            Moments m = moments(contours[0]);
            double mArea = m.m00;
            if (mArea != 0)
            {
                double mX = m.m10;
                double mY = m.m01;

                // Posisi bola
                Point2f ballPosition(mX / mArea, mY / mArea);
                circle(frame, ballPosition, 3, Scalar(0, 0, 255), -1);

                // Init start ball pos
                if (startBallPosition.x == 0 && startBallPosition.y == 0)
                {
                    startBallPosition.x = mX;
                    startBallPosition.y = mY;
                }

                // Get robot pos
                double robotX = cvRound(startBallPosition.x - mX);
                double robotY = cvRound(mY - startBallPosition.y);
                line(frame, centerOfRobot, ballPosition, Scalar(255, 0, 0));

                // Tampilkan informasi pada frame
                string posInfo = "Robot Position: (" + to_string(int(robotX / 10)) + ", " + to_string(int(robotY / 10)) + ")";
                string ballPosInfo = "Ball Position: (" + to_string(int(mX / 10)) + ", " + to_string(int(mY / 10)) + ")";

                putText(frame, posInfo, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
                putText(frame, ballPosInfo, Point(10, 60), FONT_HERSHEY_SIMPLEX, 0.7, Scalar(0, 255, 0), 2);
            }
        }

        // Show thresholded image
        imshow("Threshold", frame_thres);

        // Show original image
        imshow("Camera", frame);

        if (waitKey(1) == 'q')
        {
            break;
        }
    }
    return 0;
}