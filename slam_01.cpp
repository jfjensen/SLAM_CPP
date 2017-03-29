#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include "Eigen/Dense"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

void filter_step(VectorXd &pose, VectorXd motor_ticks, float ticks_to_mm, float robot_width)
{
	
	VectorXd new_pose = VectorXd(3);

	// cout << "motorticks: " << motor_ticks << endl;

	if (motor_ticks(0) == motor_ticks(1))
	{
		float old_theta = pose(2);
		float tot_mms = motor_ticks(0) * ticks_to_mm;
		float x = pose(0) + (tot_mms * cos(old_theta));
		float y = pose(1) + (tot_mms * sin(old_theta));
		float theta = old_theta;
		
		new_pose << x, y, theta;
	}
	else
	{
		float old_x = pose(0);
		float old_y = pose(1);
		float old_theta = pose(2);
		//cout << "So far so good..." << endl;
		float l = motor_ticks(0) * ticks_to_mm;
		float r = motor_ticks(1) * ticks_to_mm;

		float alpha = (r - l) / robot_width;
		float R = l / alpha;

		float cx = old_x - ((R + (robot_width/2)) *  sin(old_theta));
		float cy = old_y - ((R + (robot_width/2)) * -cos(old_theta));

		float theta = remainder((old_theta + alpha), (2 * M_PI));
		// cout << "theta: " << theta << endl;

		float x = cx + ((R + (robot_width/2)) *  sin(theta));
		float y = cy + ((R + (robot_width/2)) * -cos(theta));

		new_pose << x, y, theta;
	}


	pose = new_pose;
}

void filter_step_scanner(VectorXd &pose, VectorXd motor_ticks, float ticks_to_mm, float robot_width, float scanner_displacement)
{
	
	VectorXd new_pose = VectorXd(3);

	// cout << "motorticks: " << motor_ticks << endl;

	if (motor_ticks(0) == motor_ticks(1))
	{
		float old_theta = pose(2);
		float tot_mms = motor_ticks(0) * ticks_to_mm;

		float old_x = pose(0) - (scanner_displacement * cos(old_theta));
		float old_y = pose(1) - (scanner_displacement * sin(old_theta));

		float x = old_x + (tot_mms * cos(old_theta)) + (scanner_displacement * cos(old_theta));
		float y = old_y + (tot_mms * sin(old_theta)) + (scanner_displacement * sin(old_theta));
		float theta = old_theta;
		
		new_pose << x, y, theta;
	}
	else
	{

		float old_theta = pose(2);

		float old_x = pose(0) - (scanner_displacement * cos(old_theta));
		float old_y = pose(1) - (scanner_displacement * sin(old_theta));

		float l = motor_ticks(0) * ticks_to_mm;
		float r = motor_ticks(1) * ticks_to_mm;

		float alpha = (r - l) / robot_width;
		float R = l / alpha;

		float cx = old_x - ((R + (robot_width/2)) *  sin(old_theta));
		float cy = old_y - ((R + (robot_width/2)) * -cos(old_theta));

		float theta = remainder((old_theta + alpha), (2 * M_PI));
		//cout << "theta: " << theta << endl;

		float x = cx + ((R + (robot_width/2)) *  sin(theta)) + (scanner_displacement * cos(old_theta));
		float y = cy + ((R + (robot_width/2)) * -cos(theta)) + (scanner_displacement * sin(old_theta));

		new_pose << x, y, theta;
	}


	pose = new_pose;
}

void GetTickDiffs(const vector<VectorXd> &ticks, vector<VectorXd> &tick_diffs)
{
	VectorXd prev_ticks = VectorXd(2);
	prev_ticks = ticks.at(0);

	for (int i = 1; i < ticks.size(); i++)
	{
		VectorXd current_ticks = ticks.at(i);
		VectorXd diff          = current_ticks - prev_ticks;
		tick_diffs.push_back(diff);
		prev_ticks = current_ticks;

	}
}


int main(int argc, char const *argv[])
{
	string in_file_name_ = "robot4_motors.txt";
	ifstream in_file_(in_file_name_.c_str(), ifstream::in);

	string out_file_name_ = "pose.txt";
	ofstream out_file_(out_file_name_.c_str(), ofstream::out);

	string line;

	vector<VectorXd> all_ticks;

	cout << "Reading the input file..." << endl;

	while (getline(in_file_, line)) {
		istringstream iss(line);
		string M;
		iss >> M;
		int timestamp; // in ms
		int pos_left; // in ticks
		int val1,val2,val3;
		int pos_right; // in ticks
		iss >> timestamp;
		iss >> pos_left;
		iss >> val1;
		iss >> val2;
		iss >> val3;
		iss >> pos_right;
		//cout << "Left pos: " << pos_left << " Right pos: " << pos_right << endl;
		VectorXd ticks = VectorXd(2);
		ticks << pos_left, pos_right;
		all_ticks.push_back(ticks);

	}

	vector<VectorXd> tick_diffs;
	GetTickDiffs(all_ticks, tick_diffs);

	float scanner_displacement = 30.0;
	float ticks_to_mm          = 0.349;
	float robot_width          = 173.0; //150.0;
	VectorXd pose = VectorXd(3);
	pose << 1850.0,
	        1897.0,
	        (213.0 / 180.0 * M_PI);

	cout << "Filter step..." << endl;

	for(int i = 0; i < tick_diffs.size(); i++)
	{
		filter_step_scanner(pose, tick_diffs.at(i), ticks_to_mm, robot_width, scanner_displacement);
		// cout << pose << endl;
		out_file_ << "F ";
		out_file_ << pose(0) << " ";
		out_file_ << pose(1) << " ";
		out_file_ << pose(2) << endl;
	}



	if (in_file_.is_open()) {
    	in_file_.close();
  	}

  	if (out_file_.is_open()) {
    	out_file_.close();
  	}

  	cout << "Finished..." << endl;

	return 0;
}