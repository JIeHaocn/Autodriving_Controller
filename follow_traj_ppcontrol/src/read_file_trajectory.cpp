#include "follow_traj_ppcontrol/follow_traj_ppcontroller.h"

#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

const bool DEBUG=true;
const std::string INPUT_FILE="/home/ad/workspace/racecar-ws/map_trajectory.txt";
/*
 *The format of the input file is that each line is a command,
 *as an example is :"1 0 0 1".
 *The first word of each command is the x-coordinate,
 *the second word is y-coordinate and the third one is the
 *radius, the last one is the speed.
*/

int main(int argc, char **argv){
    ros::init(argc, argv, "follow_traj_ppcontrol");
    ros::NodeHandle nh;
    
    std::stringstream ss;
    try{
        // read the text from the input file and then store it in memory
	std::fstream f1(INPUT_FILE,std::fstream::in);
        if(!f1){
            if(DEBUG) ROS_INFO("error");
            throw std::string("Can not open "+INPUT_FILE);
            //if can not open the file throw error msg
        }
        ss<<f1.rdbuf();
        f1.close();
    }
    catch(std::string s){
        std::cout<<s<<std::endl;
        return -1;
    }

    std::vector<double> trajectory_points;	
    std::string x_d, y_d, speed_d;
    
    ss >> x_d;
	while(!ss.eof()){	   	  	
        ss >> y_d >> speed_d;
	
        trajectory_points.push_back(stod(x_d));
        trajectory_points.push_back(stod(y_d));
        trajectory_points.push_back(stod(speed_d));
        
        if(DEBUG) ROS_INFO("desire_x: %.3f,desire_y:%.3f,desire_speed: %.3f",stod(x_d),stod(y_d),stod(speed_d));	
	    ss >> x_d;

    }
    
    if(DEBUG) ROS_INFO("Trajectory_points_vector_dimension: %ld",trajectory_points.size());	
    DecisionPure::PPControl ppcon(nh, trajectory_points);
    ppcon.ControlSignal();

    // auto tr1 = trajectory_points.begin();
    // auto tr2 = trajectory_points.end();
    // while(tr1!=tr2)
    // {
    // 	std::cout << *tr1++ << std::endl; 
	// }

    return 0;

} //main
