#ifndef MAIN_FUNCS_H
#define MAIN_FUNCS_H
#include "opencv2/core/core.hpp"
#include "CorrLineDetector.h"
char GetCamDirection(cv::Mat & img, CorrLineDetector & corr_line);
bool InitCameraPose(CorrLineDetector ** pCorr_line, cv::Mat & start_img);
bool Exit_clean(CorrLineDetector * pCorr_line);

#endif MAIN_FUNCS_H