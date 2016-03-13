#ifndef LINEMERGER_H
#define LINEMERGER_H

#include <vector>
#include <list>
#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
using namespace std;
class LineMerger{
private:
	//variable
	cv::Mat & line_vector;
	cv::Mat & debug_figure;
	list<pair<double, vector<float> > > & list_rho_line;
	int lt;
	double collinear_dist_thres;

	// function
	double get_dist_point_line(double x, double y, const list<pair<double, vector<float>>>::iterator &);
	//bool evaluate_parallel(const list<pair<double, vector<float>>>::iterator &line0, const list<pair<double, vector<float>>>::iterator &line1);
	bool evaluate_collinear(const list<pair<double, vector<float>>>::iterator &line0, const list<pair<double, vector<float>>>::iterator &line1);
	bool evaluate_near(list<pair<double, vector<float>>>::iterator &line0, const list<pair<double, vector<float>>>::iterator &line1);
	double measure_line_dist(vector<float> & line);
	double measure_line_dist(double x0, double y0, double x1, double y1);
	double get_rho(const vector<float> & line);
	double get_rho(const float x0, const float y0, const float x1, const float y1);
	bool assert_parallel(double cos_theta_thres, const float x0, const float y0,\
		const float x1, const float y1,\
		const list<pair<double, vector<float>>>::iterator &line0);
public: 
	LineMerger(cv::Mat & line_vec, list<pair<double, vector<float> > > & list_rho_mergedline, int lenth_thres, cv::Mat & debug_figure, int im_cols);
	int Merge_lines();
};

#endif LINEMERGER_H