#ifndef ROBOT_H
#define ROBOT_H

#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include "Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class Robot
{
public:
	Robot(float ticks_to_mm, float robot_width, float scanner_displacement);
	~Robot();
	void filter_step_scanner(VectorXd &pose, VectorXd motor_ticks);
	void compute_derivative(const vector<int> & scan, vector<float> & jumps);
	void find_cylinders(const vector<int> & scan, const vector<float> & scan_deriv, vector<VectorXd> & cylinder_list);
	void compute_cartesian(const vector<VectorXd> & cylinders, vector<VectorXd> & result);
	float beam_index_to_angle(int ray_ix);
	void read(string filename);
	int size();
	void info();


	vector<VectorXd> reference_positions_;
	vector<vector<int>> scan_data_;
	vector<vector<int>> pole_indices_;
	vector<VectorXd> motor_ticks_;
	vector<VectorXd> filtered_positions_;
	vector<VectorXd> landmarks_;
	vector<VectorXd> detected_cylinders_;
	vector<VectorXd> last_ticks_;

private:
	// In previous versions, the S record included the number of scan points.
	// If so, set this to true.
	bool s_record_has_count_ = true;
	float ticks_to_mm_;
	float robot_width_;
	float scanner_displacement_;
	float min_dist_;
	float depth_jump_;
	float cylinder_offset_;
	float mounting_angle_;
};


#endif // ROBOT_H
