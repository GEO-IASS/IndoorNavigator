#include "LineMerger.h"
#include <iostream>
#include <algorithm>
#include <math.h>
#include <utility>
#include <vector>
#include <cmath>
#include <list>
#define PI 3.14159265358979323846
#define RHO_DIF_THRES (PI*5/180)//5 degree
#define MIN_DIST_PERCENT 0.1
#define COS_THETA_THRES (cos(PI*3/180))//3 degree
#define MAX_MERGED_LINE_PERCENT 1.3
#define COLLINEAR_DIST_PERCENT 0.01
//#define R_CORR_LINE_RHO_RANGE_MIN (PI*(-20)/180)
//#define R_CORR_LINE_RHO_RANGE_MAX (PI*160/180)
using namespace std;
LineMerger::LineMerger(cv::Mat & line_vec, list<pair<double, vector<float> > > & list_rho_mergedline, int lenth_thres, cv::Mat & debug_figure, int im_cols) :line_vector(line_vec), list_rho_line(list_rho_mergedline), lt(lenth_thres), debug_figure(debug_figure)
{
	collinear_dist_thres = COLLINEAR_DIST_PERCENT * im_cols;
	//cout << "LineMerger created" << endl;
}

int LineMerger::Merge_lines()
{
	vector<float> merged_line;
	vector<float> ok_line;
	
	double dist_merged_line = 0;
	double rho = 0.0;
	//list<pair<double, vector<float> > > list_rho_line;
	int line_class = 0;
	int j = 0;


	cv::MatIterator_<cv::Vec4f> it_line, it_end;
	it_line = line_vector.begin<cv::Vec4f>();
	it_end = line_vector.end<cv::Vec4f>();
	while (it_line != it_end)
	{
		if (lt < measure_line_dist((*it_line)[0], (*it_line)[1], (*it_line)[2], (*it_line)[3]))
		{
			vector<float> tmp = { (*it_line)[0], (*it_line)[1], (*it_line)[2], (*it_line)[3] };
			list_rho_line.push_back(pair<double, vector<float>>(get_rho(tmp), tmp));
		}
		++it_line;
	}
	list_rho_line.sort();
	list<pair<double, vector<float>>>::iterator line_it;
	list<pair<double, vector<float>>>::iterator next_line_it;
	list<pair<double, vector<float>>>::iterator erase_line_it;
	double debug_old_len = 0;
	int old_list_len = list_rho_line.size();
	cv::Mat debug_tmp_im = debug_figure;
	for (line_it = list_rho_line.begin(); line_it != list_rho_line.end(); line_it=next(line_it))
	{
		for (next_line_it = next(line_it); next_line_it != list_rho_line.end(); next_line_it=next(next_line_it))
		{
			//if (line_it->second[1] > 700 || line_it->second[3] > 700)
			//{
			//	if (next_line_it->second[1] > 700 || next_line_it->second[3] > 700)
			//	{
			//		debug_tmp_im = debug_figure;
			//		cv::Point2f p0((line_it->second)[0], (line_it->second)[1]);
			//		cv::Point2f p1((line_it->second)[2], (line_it->second)[3]);
			//		//if (norm(p0 - p1) > lenth_thres)
			//		//{
			//		cv::circle(debug_tmp_im, p0, 5, cv::Scalar(0, 255, 0), -1);
			//		cv::circle(debug_tmp_im, p1, 5, cv::Scalar(255, 0, 0), -1);
			//		arrowedLine(debug_tmp_im, p0, p1, cv::Scalar(255, 255, 0), 2);
			//		
			//		cv::Point2f p2((next_line_it->second)[0], (next_line_it->second)[1]);
			//		cv::Point2f p3((next_line_it->second)[2], (next_line_it->second)[3]);
			//		//if (norm(p0 - p1) > lenth_thres)
			//		//{
			//		cv::circle(debug_tmp_im, p2, 5, cv::Scalar(0, 255, 0), -1);
			//		cv::circle(debug_tmp_im, p3, 5, cv::Scalar(255, 0, 0), -1);
			//		arrowedLine(debug_tmp_im, p2, p3, cv::Scalar(255, 255, 0), 3);


			//		cout << "key line come" << abs(line_it->first - next_line_it->first) << endl;
			//	}
			//}
			if (abs(line_it->first - next_line_it->first) < RHO_DIF_THRES)
			{
				//cout << "almost parallel" << endl;
				//almost parallel line
				
				if (evaluate_collinear(line_it, next_line_it))
				{
					//cout << "collinear, "<< "rho:" << line_it->first << endl;
					debug_old_len = measure_line_dist(line_it->second);
					if (evaluate_near(line_it, next_line_it))
					{

						//cout << "merged" << endl;
						//cout << "old lenth:" << debug_old_len << "new lenth:" << measure_line_dist(line_it->second) << endl;
						erase_line_it = next_line_it;
						next_line_it = prev(next_line_it);
						list_rho_line.erase(erase_line_it);
					}
				}
			}
			else
			{
				break;
			}
		}
	}
	//cout << "list start length:"<< old_list_len << "list end length:" << list_rho_line.size() << endl;
	//for (line_it = list_rho_line.begin(); line_it != list_rho_line.end(); ++line_it)
	//{
	//	if ((line_it->second[1] > 720 || line_it->second[3] > 720))
	//	{
	//		if (line_it->first > L_CORR_LINE_RHO_RANGE_MIN && line_it->first < L_CORR_LINE_RHO_RANGE_MAX)
	//			merged_line_vector.push_back(line_it->second);
	//		if (line_it->first > -L_CORR_LINE_RHO_RANGE_MAX && line_it->first < -L_CORR_LINE_RHO_RANGE_MIN)
	//			merged_line_vector.push_back(line_it->second);
	//	}
	//}
	return 0;
}


double LineMerger::measure_line_dist(vector<float> & line)
{
	double pw0 = pow(line[0] - line[2], 2);
	double pw1 = pow(line[1] - line[3], 2);
	return sqrt(pw0 + pw1);
}
double LineMerger::measure_line_dist(double x0, double y0, double x1, double y1)
{
	double pw0 = pow(x0 - x1, 2);
	double pw1 = pow(y0 - y1, 2);
	return sqrt(pw0 + pw1);
}
double LineMerger::get_rho(const vector<float> & line)
{
	double rho = atan((line[3] - line[1]) / (line[2] - line[0]));
	//if (rho < 0)
	//{
	//	rho += PI;
	//}
	return rho;
}
double LineMerger::get_rho(const float x0, const float y0, const float x1, const float y1)
{
	double rho = atan((y1 - y0) / (x1 - x0));
	//if (rho < 0)
	//{
	//	rho += PI;
	//}
	return rho;
}

//bool LineMerger::evaluate_parallel(const list<pair<double, vector<float>>>::iterator &line0,\
//									const list<pair<double, vector<float>>>::iterator &line1)
//{
//	
//}
bool LineMerger::evaluate_collinear(const list<pair<double, vector<float>>>::iterator &line0,\
									const list<pair<double, vector<float>>>::iterator &line1)
{
	if (collinear_dist_thres > get_dist_point_line((line1->second)[0], (line1->second)[1], line0))
	{
		if (collinear_dist_thres > get_dist_point_line(line1->second[2], line1->second[3], line0))
		{
			return true;
		}
	}
	return false;
}
bool LineMerger::evaluate_near(list<pair<double, vector<float>>>::iterator & line0,\
							const list<pair<double, vector<float>>>::iterator & line1)
{
	double len_line0 = measure_line_dist(line0->second);
	double len_line1 = measure_line_dist(line1->second);
	double max_merged_line_len = 0;
	double tmp_len = 0;
	int max_line_i = 0;
	int max_line_j = 0;
	int i = 0;
	int j = 0;
	for (i = 0; i < 2; ++i)
	{
		for (j = 0; j < 2; ++j)
		{
			tmp_len = measure_line_dist(line0->second[i * 2 + 0], line0->second[i * 2 + 1], \
				line1->second[j * 2 + 0], line1->second[j * 2 + 1]);
			if (tmp_len > max_merged_line_len)
			{
				max_merged_line_len = tmp_len;
				max_line_i = i;
				max_line_j = j;
			}
		}
	}
	if (max_merged_line_len > MAX_MERGED_LINE_PERCENT * (len_line0 + len_line1))
	{
		return false;
	}
	else
	{
		if (max_merged_line_len > len_line0)
		{
			if (max_merged_line_len > len_line1)
			{
				// new merged line is the longest.
				line0->second[0] = line0->second[max_line_i * 2 + 0];
				line0->second[1] = line0->second[max_line_i * 2 + 1];
				line0->second[2] = line1->second[max_line_j * 2 + 0];
				line0->second[3] = line1->second[max_line_j * 2 + 1];
			}
		}
		else // longest line is line0 or line1.
		{
			if (len_line0 > len_line1)
			{//line 0 is the longest.

			}
			else
			{ // line 1 is the longest.
				line0->second = line1->second;
			}
		}

		return true;
	}
}

bool LineMerger::assert_parallel(double cos_theta_thres,\
								const float x0,	const float y0,\
								const float x1,	const float y1,\
								const list<pair<double, vector<float>>>::iterator &line0)
{
	double dx_l = x1-x0;
	double dy_l = y1-y0;
	double dx_r = line0 -> second[2] - line0 -> second[0] ;
	double dy_r = line0 -> second[3] - line0 -> second[1];
	double cos_theta = (dx_l*dx_r + dy_l*dy_r) / \
		sqrt((pow(dx_l, 2) + pow(dy_l, 2)) * (pow(dx_r, 2) + pow(dy_r, 2)));
	if (cos_theta > 1)
	{
		cout << "ERROR: cos_theta="<< cos_theta << "> 1" << endl;
		throw;
	}
	if (abs(cos_theta) > cos_theta_thres)
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*
* two point line format. Calculate the dist from a point(x0, y0) to line
*/
double LineMerger::get_dist_point_line(double x, double y, const list<pair<double, vector<float>>>::iterator & line)
{
	double x0 = line->second[0];
	double y0 = line->second[1];
	double x1 = line->second[2];
	double y1 = line->second[3];
	return abs((y1 - y0)*x - (x1 - x0)*y + x1*y0 - y1*x0) / sqrt(pow(y1 - y0, 2) + pow(x1 - x0, 2));
}