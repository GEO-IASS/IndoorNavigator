#include "opencv2\core\core.hpp"
#include "opencv2\core\utility.hpp"
#include "opencv2\imgproc\imgproc.hpp"

#include <algorithm>

#include "LineMerger.h"
#include "CorrLineDetector.h"

#define PI 3.14159265358979323846
#define L_CORR_LINE_RHO_RANGE_MIN (PI*20/180)
#define L_CORR_LINE_RHO_RANGE_MAX (PI*70/180)

using namespace cv;
CorrLineDetector::CorrLineDetector(cv::Mat & img_ref, int length_thres) :img(img_ref), length_thres(length_thres)
{

}


bool CorrLineDetector::SetImg(cv::Mat & img_ref)
{
	this->img = img_ref;
	return true;
}

bool CorrLineDetector::InitCamPose()
{
	unsigned int now_line[2][4] = { {0}, {0} };
	if (!GetCorrLinePose(now_line))
	{
		return false;
	}
	else
	{
		//std::copy(std::begin(now_line), std::end(now_line), std::begin(start_line));
		memcpy(this->start_line, now_line, sizeof(unsigned int) * 2* 4);
		int left_x = min(now_line[0][0], now_line[0][2]);
		int right_x = max(now_line[1][0], now_line[1][2]);
		this->current_center_x = (left_x + right_x) / 2;
		this->start_center_x = this->current_center_x;
	}
}

bool CorrLineDetector::GetCorrLinePose(unsigned int now_line[2][4])
{
	list<pair<double, vector<float> > > list_rho_mergedline;
	vector < vector<float> > left_corr_line;
	vector < vector<float> > right_corr_line;
	//blur the img. 
	GaussianBlur(this->img, this->img, Size(3, 3), 0, 0);
	//Edge detection.
	Canny(this->img, this->img, 40, 200, 3);
	//LSD.
	// // Create and LSD detector with standard or no refinement.
	Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
	Mat lines_std;
	// //Detect the lines
	ls->detect(this->img, lines_std);
	// // Show found lines.
	Mat line_figure(this->img.rows, this->img.cols, CV_8UC3);
	ls->drawSegments(line_figure, lines_std);
	// //Merge the line segment.
	LineMerger mg(lines_std, list_rho_mergedline, this->length_thres, line_figure, this->img.cols);
	mg.Merge_lines();
	// //Select the key line.
	list<pair<double, vector<float> > >::iterator line_it;
	for (line_it = list_rho_mergedline.begin(); line_it != list_rho_mergedline.end(); ++line_it)
	{
		if ((line_it->second[1] > 460 || line_it->second[3] > 460))
		{
			if (line_it->first > L_CORR_LINE_RHO_RANGE_MIN && line_it->first < L_CORR_LINE_RHO_RANGE_MAX)
			{
				left_corr_line.push_back(line_it->second);
			}
			if (line_it->first > -L_CORR_LINE_RHO_RANGE_MAX && line_it->first < -L_CORR_LINE_RHO_RANGE_MIN)
			{
				right_corr_line.push_back(line_it->second);
			}
		}
	}

	if (left_corr_line.size() == 0 || right_corr_line.size() == 0)
	{
		return false;
	}
	//Pick out the corridor line.
	vector < vector<float> >::iterator it_left = left_corr_line.begin();
	int max_x = 0;
	do{
		if ((*it_left)[0] > max_x || (*it_left)[2] > max_x)
		{

			max_x = (*it_left)[0] > (*it_left)[2] ? (*it_left)[0] : (*it_left)[2];
			now_line[0][0] = (*it_left)[0];
			now_line[0][1] = (*it_left)[1];
			now_line[0][2] = (*it_left)[2];
			now_line[0][3] = (*it_left)[3];
		}
		++it_left;
	} while (it_left != left_corr_line.end());

	vector < vector<float> >::iterator it_right = right_corr_line.begin();
	unsigned int min_x = (*it_right)[0];
	do{
		if ((*it_right)[0] < min_x || (*it_right)[2] < min_x)
		{
			min_x = (*it_right)[0] < (*it_right)[2] ? (*it_right)[0] : (*it_right)[2];
			now_line[1][0] = (*it_right)[0];
			now_line[1][1] = (*it_right)[1];
			now_line[1][2] = (*it_right)[2];
			now_line[1][3] = (*it_right)[3];
		}
		++it_right;
	} while (it_right != right_corr_line.end());
	return true;
}