#include <iostream>
#include <string>
#include <sstream>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "LineMerger.h"
#include "CorrLineDetector.h"
#include "main_funcs.h"
#define PI 3.14159265358979323846
#define L_CORR_LINE_RHO_RANGE_MIN (PI*20/180)
#define L_CORR_LINE_RHO_RANGE_MAX (PI*70/180)
using namespace cv;

int main(int argc, char** argv)
{
	vector < vector<float> > lines_vector;
    string in, out_file_name;
	list<pair<double, vector<float> > > list_rho_mergedline;
	if (argc < 4)
	{
		cout << "at least 4 arg" << endl;
		return -1;
	}
	in = argv[1];
	out_file_name = argv[2];
	istringstream ss(argv[3]);
	int lenth_thres;
	char cam_bias = '\0';
	ss >> lenth_thres;
	cout << in << endl;
	//Mat hsv, RGB_im;
	//RGB_im = imread(in);
	//cvtColor(RGB_im, hsv, CV_RGB2HSV);
	//vector<Mat> channels;
	//split(hsv, channels);
	//Mat image = channels[2];
	//cout << "image size:" << image.size() << endl;
	Mat image = imread(in, IMREAD_GRAYSCALE);
	//imshow("gray figure:", image);
	//waitKey(1);
	//CorrLineDetector *pCorr_line = nullptr;
	//InitCameraPose(&pCorr_line, image);
	////pCorr_line->DEBUG_print_line();
	///*Figure Test*/
	////image = imread("./corr_c_l_low.jpg", IMREAD_GRAYSCALE);
	////cout<<"corr_c_l_low: "<< GetCamDirection(image, *pCorr_line)<< endl;
	////image = imread("./corr_c_r_low.jpg", IMREAD_GRAYSCALE);
	////cout << "corr_c_r_low: " << GetCamDirection(image, *pCorr_line) << endl;
	////
	///*Video test Start*/
	//VideoCapture cap("corr_logitech.wmv");
	//if (!cap.isOpened())
	//{
	//	cout << "Capture could not be opened successfully" << endl;
	//	return -1;
	//}
	//namedWindow("Video");
	//while (char(waitKey(1)) != 'q' && cap.isOpened())
	//{
	//	cap >> image;
	//	if (image.empty())
	//	{
	//		cout << "Video over" << endl;
	//		break;
	//	}
	//	imshow("Video", image);
	//	cout << "Direction :" << GetCamDirection(image, *pCorr_line) << endl;
	//}
	///*Video test Over*/
	//
	//Exit_clean(pCorr_line);
	//cout << "press any key to exit" << endl;
	//char exit;
	//cin >> exit;
	//GetCamDirection(image);

	GaussianBlur(image, image, Size(3, 3), 0, 0);
    Canny(image, image, 20, 80, 3); // Apply canny edge
	//imshow("Canny",image);
	//waitKey(6);
	//char tmp;
	//cin >> tmp;
    // Create and LSD detector with standard or no refinement.
    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
    double start = double(getTickCount());
    Mat lines_std;
    // Detect the lines
    ls->detect(image, lines_std);
	// Show found lines.
	Mat line_figure(image.rows, image.cols, CV_8UC3);
	ls->drawSegments(line_figure, lines_std);
	// Merge the lines.
	LineMerger mg(lines_std, list_rho_mergedline, lenth_thres, line_figure, image.cols);
	mg.Merge_lines();
	// Select the key line.
	list<pair<double, vector<float> > >::iterator line_it;
	for (line_it = list_rho_mergedline.begin(); line_it != list_rho_mergedline.end(); ++line_it)
	{
		if ((line_it->second[1] > 460 || line_it->second[3] > 460))
		{
			if (line_it->first > L_CORR_LINE_RHO_RANGE_MIN && line_it->first < L_CORR_LINE_RHO_RANGE_MAX)
				lines_vector.push_back(line_it->second);
			if (line_it->first > -L_CORR_LINE_RHO_RANGE_MAX && line_it->first < -L_CORR_LINE_RHO_RANGE_MIN)
				lines_vector.push_back(line_it->second);
		}
	}
	// Print time used.
	double duration_ms = (double(getTickCount()) - start) * 1000 / getTickFrequency();
	cout << "It took " << duration_ms << " ms." << endl;
	// Show found lines.
	line_figure = Mat(image.rows, image.cols, CV_8UC3);
	ls->drawSegments(line_figure, lines_std);
	//draw start point and end point of every line.
	vector<vector<float>>::iterator vec_it;
	for (vec_it = lines_vector.begin(); vec_it != lines_vector.end(); ++vec_it)
	{
		Point2f p0((*vec_it)[0], (*vec_it)[1]);
		Point2f p1((*vec_it)[2], (*vec_it)[3]);
		//if (norm(p0 - p1) > lenth_thres)
		//{
			circle(line_figure, p0, 5, Scalar(0, 255, 0), -1);
			circle(line_figure, p1, 5, Scalar(255, 0, 0), -1);
			arrowedLine(line_figure, p0, p1, Scalar(255, 255, 0), 1);
		//}
	}
    //imshow("Standard refinement", drawnLines);
    //waitKey();
	imwrite(out_file_name, line_figure);
    return 0;
}