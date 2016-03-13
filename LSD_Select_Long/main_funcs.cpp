#include "main_funcs.h"
#include <algorithm>
#define LSD_LINE_LENGTH_THRES 20
#define CENTER_LEFT_LIMIT 0.9
#define CENTER_RIGHT_LIMIT 1.1
using namespace std;
char GetCamDirection(cv::Mat & img, CorrLineDetector & corr_line)
{
	unsigned int now_line[2][4] = { { 0 }, { 0 } };
	corr_line.SetImg(img); // update image
	if (!corr_line.GetCorrLinePose(now_line))
	{
		return '\0';
	}
	else
	{
		//std::copy(std::begin(now_line), std::end(now_line), std::begin(start_line));
		memcpy(corr_line.current_line, now_line, sizeof(unsigned int) * 2 * 4);
		int left_x = min(now_line[0][0], now_line[0][2]);
		int right_x = max(now_line[1][0], now_line[1][2]);
		corr_line.current_center_x = (left_x + right_x) / 2;
		if (corr_line.current_center_x < CENTER_LEFT_LIMIT*corr_line.start_center_x)
		{
			return 'r';
		}
		else if (corr_line.current_center_x > CENTER_RIGHT_LIMIT*corr_line.start_center_x)
		{
			return 'l';
		}
		else
		{
			return 'c';
		}
	}
}

bool InitCameraPose(CorrLineDetector ** pCorr_line, cv::Mat & start_img)
{
	*pCorr_line = new CorrLineDetector(start_img, LSD_LINE_LENGTH_THRES);
	if (!(*pCorr_line)->InitCamPose())
	{
		return false;
	}
	return true;
}

bool Exit_clean(CorrLineDetector * pCorr_line)
{
	delete pCorr_line;
	return true;
}