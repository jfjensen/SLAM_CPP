#include <fstream>
#include <iostream>
#include <sstream>
#include <stdlib.h>
#include <vector>
#include "Eigen/Dense"
#include "robot.h"


using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;




int main(int argc, char const *argv[])
{
	string scan_file_name = "robot4_scan.txt";
	string ref_file_name = "robot4_reference.txt";
	string ticks_file_name = "robot4_motors.txt";

	string out_file_name = "pose.txt";
	ofstream out_file(out_file_name.c_str(), ofstream::out);

	string cylinder_file_name = "cylinder.txt";
	ofstream cylinder_file(cylinder_file_name.c_str(), ofstream::out);
	
	float scanner_displacement = 30.0;
	float ticks_to_mm          = 0.349;
	float robot_width          = 173.0; //150.0;

	Robot robot = Robot(ticks_to_mm, robot_width, scanner_displacement);
	robot.read(scan_file_name);
	robot.read(ref_file_name);
	robot.read(ticks_file_name);
	robot.info();

	VectorXd pose = VectorXd(3);
	pose << 1850.0,
	        1897.0,
	        (213.0 / 180.0 * M_PI);

	cout << "Filter step..." << endl;

	for(int i = 0; i < robot.motor_ticks_.size(); i++)
	{
		robot.filter_step_scanner(pose, robot.motor_ticks_.at(i));
		//cout << pose(0) << " " << pose(1) << " " << pose(2) << endl;
		out_file << "F ";
		out_file << pose(0) << " ";
		out_file << pose(1) << " ";
		out_file << pose(2) << endl;
	}

	cout << "Scan derivative..." << endl;

	for (int ix = 0; ix < robot.scan_data_.size(); ix++)
	{
			// int scan_no = 235;
		vector<int> scan = robot.scan_data_.at(ix);

		vector<float> jumps;
		robot.compute_derivative(scan, jumps);
		// cout << "Jumps: " << endl;
		// for (int i = 0; i < jumps.size(); i++)
		// {
		// 	scan_deriv_file << jumps.at(i) << " ";
		// }
		// scan_deriv_file << endl;

		vector<VectorXd> cylinder_list;
		robot.find_cylinders(scan, jumps, cylinder_list);
		for (int i = 0; i < cylinder_list.size(); i++)
		{
			cout << cylinder_list.at(i) << " ";
		}
		cout << endl;

		vector<VectorXd> cartesian_cylinders;
		robot.compute_cartesian(cylinder_list, cartesian_cylinders);

		cylinder_file << "D C ";
		for (int i = 0; i < cartesian_cylinders.size(); i++)
		{
			VectorXd coords = VectorXd(2);
			coords = cartesian_cylinders.at(i);
			cylinder_file << coords(0) << " ";
			cylinder_file << coords(1) << endl;
		}

	}


  	cout << "Finished..." << endl;

  	if (out_file.is_open()) {
    	out_file.close();
  	}

  	if (cylinder_file.is_open()) {
    	cylinder_file.close();
  	}

	return 0;
}