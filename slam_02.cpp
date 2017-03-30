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

  	cout << "Finished..." << endl;

  	if (out_file.is_open()) {
    	out_file.close();
  	}

	return 0;
}