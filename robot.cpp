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

Robot::Robot(float ticks_to_mm, float robot_width, float scanner_displacement)
{
	cout << "Creating robot instance..." << endl;
	reference_positions_.clear();
	scan_data_.clear();
	pole_indices_.clear();
	motor_ticks_.clear();
	filtered_positions_.clear();
	landmarks_.clear();
	detected_cylinders_.clear();
	last_ticks_.clear();

	ticks_to_mm_ = ticks_to_mm;
	robot_width_ = robot_width;
	scanner_displacement_ = scanner_displacement;

	min_dist_ = 20.0;
	depth_jump_ = 100.0;
	cylinder_offset_ = 90.0;
	mounting_angle_ = -0.06981317007977318;
}

Robot::~Robot()
{
	cout << "Destroying robot instance..." << endl;
}

void Robot::read(string file_name)
{

	cout << "Reading the input file..." << endl;

	ifstream in_file(file_name.c_str(), ifstream::in);

	string line;

	vector<VectorXd> all_ticks;

	bool first_reference_positions = true;
    bool first_scan_data = true;
    bool first_pole_indices = true;
    bool first_motor_ticks = true;
    bool first_filtered_positions = true;
    bool first_landmarks = true;
    bool first_detected_cylinders = true;

    VectorXd prev_ticks;

	while (getline(in_file, line)) {
		istringstream iss(line);
		char datatype;
		iss >> datatype;
		//cout << "Datatype: " << datatype << endl;

		switch(datatype)
		{
			// P is the reference position.
	        // File format: P timestamp[in ms] x[in mm] y[in mm]
	        case 'P':
	        {
	            if (first_reference_positions)
	            {
	                reference_positions_.clear();
	                first_reference_positions = false;
	            }

	            int timestamp;
	            iss >> timestamp;
	            
	            VectorXd position = VectorXd(2);
	            float px, py;
	            iss >> px;
	            iss >> py;
	            position << px, py;
	            reference_positions_.push_back(position);
	            //cout << "Reference Positions size: " << reference_positions_.size() << endl;
	            break;
	        }

	        // S is the scan data.
	        // File format:
	        // S timestamp[in ms] distances[in mm] ...
	        // Or, in previous versions (set s_record_has_count to True):
	        // S timestamp[in ms] count distances[in mm] ...
	        case 'S':
	        {
	            if (first_scan_data)
	            {
	            	scan_data_.clear();
	                first_scan_data = false;
	            }

	            vector<int> data;
	            if (s_record_has_count_)
	            {
	            	
	            	int timestamp, count;
	            	iss >> timestamp;
	            	iss >> count;
	            	//cout << "Time: " << timestamp << " count: " << count << endl;

	            	int term;
	            	
	            	while(iss >> term)
				    {
				        //cout << term << " ";
				        data.push_back(term);
				    }
	            	
	            }
	            else
	            {
	            	int timestamp;
	            	iss >> timestamp;
	            	//cout << "Time: " << timestamp << endl;

	            	int term;
	            	
	            	while(iss >> term)
				    {
				        //cout << term << " ";
				        data.push_back(term);
				    }
	                
	            }
	            scan_data_.push_back(data);
	            //cout << endl;
	            //cout << "Scan data size: " << scan_data_.size() << endl;
	            break;
	        }

	        // I is indices of poles in the scan.
			// The indices are given in scan order (counterclockwise).
			// -1 means that the pole could not be clearly detected.
			// File format: I timestamp[in ms] index ...
	        case 'I':
	        {
	            if (first_pole_indices)
	            {
	                pole_indices_.clear();
	                first_pole_indices = false;
	            }

	            int timestamp;
	            iss >> timestamp;
	            
	    		vector<int> indices;

	            int term;
	            	
            	while(iss >> term)
			    {
			        //cout << term << " ";
			        indices.push_back(term);
			    }

	            pole_indices_.push_back(indices);
	            //cout << "Pole Indices size: " << pole_indices_.size() << endl;
	            break;
	        }

	        // M is the motor data.
         	// File format: M timestamp[in ms] pos[in ticks] tachoCount[in ticks] acceleration[deg/s^2] rotationSpeed[deg/s] ...
         	// (4 values each for: left motor, right motor, and third motor (not used)).
         	// Note that the file contains absolute ticks, but motor_ticks contains the increments (differences).

	        case 'M':
	        {
	            if (first_motor_ticks)
	            {
	                motor_ticks_.clear();
	                first_motor_ticks = false;
	                prev_ticks = VectorXd(2);

	                int timestamp; // in ms
		            iss >> timestamp;
		            
					int pos_left; // in ticks
					int val1,val2,val3;
					int pos_right; // in ticks
					iss >> pos_left;
					iss >> val1;
					iss >> val2;
					iss >> val3;
					iss >> pos_right;

					prev_ticks << pos_left, pos_right;
					break;
	            }

	            int timestamp; // in ms
	            iss >> timestamp;
	            
				int pos_left; // in ticks
				int val1,val2,val3;
				int pos_right; // in ticks
				iss >> pos_left;
				iss >> val1;
				iss >> val2;
				iss >> val3;
				iss >> pos_right;
				//cout << "Left pos: " << pos_left << " Right pos: " << pos_right << endl;
				VectorXd current_ticks = VectorXd(2);
				current_ticks << pos_left, pos_right;
				VectorXd tick_diffs = VectorXd(2);
				tick_diffs = current_ticks - prev_ticks;				
				prev_ticks = current_ticks;
				//cout << "Left pos: " << tick_diffs(0) << " Right pos: " << tick_diffs(1) << endl;
				motor_ticks_.push_back(tick_diffs);
	    		
	            //cout << "Motor ticks size: " << motor_ticks_.size() << endl;
	            break;
	        }

	        // F is filtered trajectory. No time stamp is used.
            // File format: F x[in mm] y[in mm] heading[in radians]
			case 'F':
	        {
	            if (first_filtered_positions)
	            {
	                filtered_positions_.clear();
	                first_filtered_positions = false;
	            }

	                      
	    		VectorXd position;
	    		float px, py, heading;
	    		iss >> px;
	    		iss >> py;
	    		iss >> heading;
	    		position << px, py, heading;
	            
	            filtered_positions_.push_back(position);
	            //cout << "Pole Indices size: " << pole_indices_.size() << endl;
	            break;
	        }

	        // L is landmark. This is actually background information, independent
          	// of time.
          	// File format: L <type> info...
          	// Supported types:
          	// Cylinder: L C x y diameter.
          	case 'L':
	        {
	            if (first_landmarks)
	            {
	                landmarks_.clear();
	                first_landmarks = false;
	            }
	                      
	    		char type;
	    		iss >> type;

	    		if (type=='C')
	    		{
					VectorXd landmark;
					float px, py, diameter;
					iss >> px;
					iss >> py;
					iss >> diameter;
					landmark << px, py, diameter;

					landmarks_.push_back(landmark);

	    		}
	            //cout << "Pole Indices size: " << pole_indices_.size() << endl;
	            break;
	        }

	        // D is detected landmarks (in each scan).
         	// File format: D <type> info...
         	// Supported types:
         	// Cylinder: D C x y x y ...
         	case 'D':
         	{
         		char type;
         		iss >> type;

         		if (type == 'C')
         		{
         			if (first_detected_cylinders)
         			{
         				detected_cylinders_.clear();
         				first_detected_cylinders = false;
         			}

         			float px, py;

	            	while(iss >> px and iss >> py)
				    {
				        //cout << term << " ";
				        VectorXd position = VectorXd(2);
				        position << px, py;
				        detected_cylinders_.push_back(position);
				    }

         		}
         	}

		}

	}

	if (in_file.is_open())
	{
    	in_file.close();
  	}



}

void Robot::compute_derivative(const vector<int> & scan, vector<float> & jumps)
{
	jumps.push_back(0.0);

	for (int i = 1; i < scan.size()-1; i++)
	{
		int l = scan.at(i-1);
		int r = scan.at(i+1);

		if (l > min_dist_ and r > min_dist_)
		{
			float deriv = (float)(r-l)/2.0;
			jumps.push_back(deriv);

		}
		else
		{
			jumps.push_back(0.0);
		}
	}

	jumps.push_back(0.0);
		cout << "hello" << endl;

}

void Robot::find_cylinders(const vector<int> & scan, const vector<float> & scan_deriv, vector<VectorXd> & cylinder_list)
{
	bool on_cylinder = false;
	string direction = "Left";
	float sum_ray = 0.0;
	float sum_depth = 0.0;
	int rays = 0;
	bool discard = false;

	for (int i = 0; i < scan_deriv.size(); i++)
	{
		float current_der = scan_deriv.at(i);
		if (fabs(current_der) > depth_jump_)
		{
			if ((on_cylinder) and (direction == "Left"))
			{
				if (current_der < 0.0)
				{
					discard = true;
				}
				else
				{
					on_cylinder = false;
					float average_ray   = sum_ray / rays;
					float average_depth = sum_depth / rays;

					VectorXd position = VectorXd(2);
					position << average_ray, average_depth;

					cylinder_list.push_back(position);
					sum_ray = 0.0;
					sum_depth = 0.0;
					rays = 0;
				}
			}
			if ((not on_cylinder) and (current_der < 0.0))
			{
				on_cylinder = true;
				direction = "Left";
			}
		}
		if (scan.at(i) <= min_dist_)
		{
			discard = true;
		}
		if ((on_cylinder) and (scan.at(i) > min_dist_))
		{
			rays++;
			sum_ray += i;
			sum_depth += scan.at(i);
		}
		if (discard)
		{
			sum_ray = 0.0;
			sum_depth = 0.0;
			rays = 0;
			discard = false;
		}
	}
}

float Robot::beam_index_to_angle(int ray_ix)
{
	return ((float)ray_ix - 330.0) * 0.006135923151543 + mounting_angle_;
}


void Robot::compute_cartesian(const vector<VectorXd> & cylinders, vector<VectorXd> & result)
{
	for (int i = 0; i < cylinders.size(); i++)
	{
		VectorXd c = VectorXd(2);
		c  = cylinders.at(i);
		int ray_ix = c(0);
		float rng = c(1);
		float ray_angle = beam_index_to_angle(ray_ix);
		float x = cos(ray_angle) * (rng + cylinder_offset_);
		float y = sin(ray_angle) * (rng + cylinder_offset_);

		VectorXd coords = VectorXd(2);
		coords << x, y;

		result.push_back(coords);
	}
}


int Robot::size()
{
	int i = 0;
	return i;
}

void Robot::info()
{
	cout << "Reference positions size: " << reference_positions_.size() << endl;
	cout << "Scan data size: " << scan_data_.size() << endl;
	cout << "Pole indices size: " << pole_indices_.size() << endl;
	cout << "Motor ticks size: " << motor_ticks_.size() << endl;
	cout << "Filtered trajectory size: " << filtered_positions_.size() << endl;
	cout << "Landmark size: " << landmarks_.size() << endl;
	cout << "Detected landmarks: " << detected_cylinders_.size() << endl;

	return;
}

void Robot::filter_step_scanner(VectorXd &pose, VectorXd motor_ticks)
{
	
	VectorXd new_pose = VectorXd(3);

	// cout << "motorticks: " << motor_ticks << endl;

	if (motor_ticks(0) == motor_ticks(1))
	{
		float old_theta = pose(2);
		float tot_mms = motor_ticks(0) * ticks_to_mm_;

		float old_x = pose(0) - (scanner_displacement_ * cos(old_theta));
		float old_y = pose(1) - (scanner_displacement_ * sin(old_theta));

		float x = old_x + (tot_mms * cos(old_theta)) + (scanner_displacement_ * cos(old_theta));
		float y = old_y + (tot_mms * sin(old_theta)) + (scanner_displacement_ * sin(old_theta));
		float theta = old_theta;
		
		new_pose << x, y, theta;
	}
	else
	{

		float old_theta = pose(2);

		float old_x = pose(0) - (scanner_displacement_ * cos(old_theta));
		float old_y = pose(1) - (scanner_displacement_ * sin(old_theta));

		float l = motor_ticks(0) * ticks_to_mm_;
		float r = motor_ticks(1) * ticks_to_mm_;

		float alpha = (r - l) / robot_width_;
		float R = l / alpha;

		float cx = old_x - ((R + (robot_width_/2)) *  sin(old_theta));
		float cy = old_y - ((R + (robot_width_/2)) * -cos(old_theta));

		float theta = remainder((old_theta + alpha), (2 * M_PI));
		//cout << "theta: " << theta << endl;

		float x = cx + ((R + (robot_width_/2)) *  sin(theta)) + (scanner_displacement_ * cos(old_theta));
		float y = cy + ((R + (robot_width_/2)) * -cos(theta)) + (scanner_displacement_ * sin(old_theta));

		new_pose << x, y, theta;
	}


	pose = new_pose;
}
