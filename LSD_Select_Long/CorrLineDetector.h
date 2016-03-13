#ifndef CORRLINEDETECTOR_H
#define CORRLINEDETECTOR_H
#include <iostream>
#include "opencv2/core/core.hpp"
class CorrLineDetector{
private:
	//variable
	cv::Mat & img;
	int length_thres;
	unsigned int start_line[2][4];
	//function
public:
	//variable
	unsigned int current_line[2][4];
	unsigned int start_center_x;
	unsigned int current_center_x;
	//function
	CorrLineDetector(cv::Mat & img_ref, int length_thres);
	bool InitCamPose();
	bool SetImg(cv::Mat & img_ref);
	bool GetCorrLinePose(unsigned int now_line[2][4]);
	void DEBUG_print_line()
	{
		int i = 0;
		int j = 0;
		for (i=0; i<2; ++i)
		{
			for (j=0; j<4; ++j)
			{
				std::cout << (this->start_line)[i][j] << ',';
			}
			std::cout << std::endl;
		}
	}

};

#endif CORRLINEDETECTOR_H