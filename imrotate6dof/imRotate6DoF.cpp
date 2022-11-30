/////////////////////////////////////////////////////////////////////////////
// Name:         imRotate6DoF.cpp
// Description:  Image translation and rotation in 6 DoF
// Author:       Yan Naing Aye
// Ref: https://github.com/yan9a/cecv
/////////////////////////////////////////////////////////////////////////////

#include <cstdio>
#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

#define IM_CANVAS_WIDTH 1024
#define IM_CANVAS_HEIGHT 600

void Process(Mat& img, float x, float y, float z, float yaw, float pitch, float roll, float brightness, float contrast);
Mat Perspective(Size iSize, Mat R, Mat T);
//-----------------------------------------------------------------------------
int main(int argc, char **argv)
{
    Mat img;
    img = imread("./thiri.jpg", 1);
    if (!img.data)
    {
		// fail to read img
		img = Mat::zeros(400, 125, CV_8UC3);
        printf("No image data \n");
        return -1;
    }

    // image width and height are recommended to be between 50 and 600 pixels
    // rotate image
    Process(img,0,0,0,20,15,10,0,0);

    namedWindow("Display Image", WINDOW_AUTOSIZE);
    imshow("Display Image", img);
    waitKey(0);
    return 0;
}
//-----------------------------------------------------------------------------
// get perspective matrix 
// inputs
//   Size iSize = Size(w,h) for input image
//   Mat R = [a;b;c] = 3x1 matrix
//       a = yaw rotation in degree (alpha -180 - 180 deg)
//       b = pitch rotation in degree (beta -80 - 80 deg)
//       c = roll rotation in degree (gamma -80 - 80 deg)
//   Mat T = [x;y;z] = 3x1 matrix
//       x = x translation
//       y = y translation
//       z = z translation
// outputs
//   return matrix = 3x3 perspective transform matrix
Mat Perspective(Size iSize, Mat R, Mat T)
{
	// 1024 pixels = 155 mm, at nominal distance
	float sfx = 6.6; // scale factor along x-axis
	float sfy = 6.6; // scale factor along y-axis
	float D0 = 200; // nominal distance of the screen from the camera (viewer)
	float w = float(iSize.width - 1) / sfx;
	float h = float(iSize.height - 1) / sfy;
	float cx = IM_CANVAS_WIDTH/2; // centre of the output image
	float cy = IM_CANVAS_HEIGHT/2; // centre of the output image

	// corners of the input image to get 3x4 matrix
	Mat C1 = (Mat_<float>(3, 1) << -w / 2.0, -h / 2.0, 0.0);
	Mat C2 = (Mat_<float>(3, 1) << w / 2.0, -h / 2.0, 0.0);
	Mat C3 = (Mat_<float>(3, 1) << w / 2.0, h / 2.0, 0.0);
	Mat C4 = (Mat_<float>(3, 1) << -w / 2.0, h / 2.0, 0.0);
	Mat V;
	hconcat(C1, C2, V);
	hconcat(V, C3, V);
	hconcat(V, C4, V);

	// rotate
	const float PI = 2 * acos(0.0);//get PI
	R *= PI / 180.0; // convert to radian
	float alpha = R.at<float>(0, 0);
	float beta = R.at<float>(1, 0);
	float gamma = R.at<float>(2, 0);

	Mat R3 = Mat::zeros(3, 3, CV_32F); //Mat_<float>(3,3) << cos(gamma),sin(gamma),0.0,sin(gamma),cos(gamma),0.0,0.0,0.0,1.0);
	R3.at<float>(0, 0) = cos(alpha) * cos(beta);
	R3.at<float>(0, 1) = cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma);
	R3.at<float>(0, 2) = cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma);

	R3.at<float>(1, 0) = sin(alpha) * cos(beta);
	R3.at<float>(1, 1) = sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma);
	R3.at<float>(1, 2) = sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma);

	R3.at<float>(2, 0) = -sin(beta);
	R3.at<float>(2, 1) = cos(beta) * sin(gamma);
	R3.at<float>(2, 2) = cos(beta) * cos(gamma);

	V = R3 * V;

	// projection on display and translation
	for (int i = 0; i < 4; i++) { // for ecah point
		V.at<float>(2, i) += T.at<float>(2, 0); // translate Z first to scale accordingly
		V.at<float>(0, i) *= D0 / (D0 + V.at<float>(2, i));
		V.at<float>(1, i) *= D0 / (D0 + V.at<float>(2, i));
		// translate x and y after scaling
		V.at<float>(0, i) += T.at<float>(0, 0); // x translation
		V.at<float>(1, i) += T.at<float>(1, 0); // y translation
		V.at<float>(2, i) = 1.0; // for translation
	}

	// convert 3D to 2D, scale and shift
	Mat F = (Mat_<float>(2, 3) << sfx, 0.0, cx, 0.0, sfy, cy); // 2x4
	Mat V2 = F * V; // V2 is 2x4 matrix for 4 vertices
	// cout<< "V2 = "<< V2 <<endl;

	vector<Point2f> quad1;
	vector<Point2f> quad2;

	quad1.push_back(Point2f(0, 0));
	quad1.push_back(Point2f(iSize.width - 1, 0));
	quad1.push_back(Point2f(iSize.width - 1, iSize.height - 1));
	quad1.push_back(Point2f(0, iSize.height - 1));

	quad2.push_back(Point2f(V2.at<float>(0, 0), V2.at<float>(1, 0)));
	quad2.push_back(Point2f(V2.at<float>(0, 1), V2.at<float>(1, 1)));
	quad2.push_back(Point2f(V2.at<float>(0, 2), V2.at<float>(1, 2)));
	quad2.push_back(Point2f(V2.at<float>(0, 3), V2.at<float>(1, 3)));

	Mat p = getPerspectiveTransform(quad1, quad2);
	// cout<<p<<endl;
	// cout<< iSize.width << " " << iSize.height << endl;
	return p;
}
//-----------------------------------------------------------------------------
// Translate and rotate in 6 DoF, and then modify brightness and contrast
// Inputs (recommended range):
//  x = translation along x axis ( -75 to 75 mm )
//  y = translation along y axis ( -40 to 40 mm )
//  z = translation along z axis ( -100 to 100 mm )
//  yaw = rotation angle in degree about x axis - (alpha -180 - 180 deg)
//  pitch = rotation angle in degree about y axis - (beta -80 - 80 deg)
//  roll = rotation angle in degree about z axis - (gamma -80 - 80 deg)
//  brightness = brightness +/- ( -100 to 100 )
//  contrast = contrast +/- ( -100 to 100 )
// Output:
//  img = by ref
void Process(Mat& img, float x, float y, float z, float yaw, float pitch, float roll, float brightness, float contrast)
{
	Mat R = (Mat_<float>(3, 1) << yaw, pitch, roll); // rotation matrix
	Mat T = (Mat_<float>(3, 1) << x, y, z); // translation matrix
	Mat p = Perspective(img.size(), R, T);
	Size oSize = Size(IM_CANVAS_WIDTH, IM_CANVAS_HEIGHT);
	warpPerspective(img, img, p, oSize, INTER_LINEAR, BORDER_CONSTANT, Scalar(128, 128, 128));

	// brightness and contrast
	double a_contrast = (double(contrast)+150.0)/150.0;
	double b_brightness = brightness;
	img.convertTo(img, -1, a_contrast, b_brightness);
}
//-----------------------------------------------------------------------------
