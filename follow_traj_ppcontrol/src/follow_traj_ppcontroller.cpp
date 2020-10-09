#include "follow_traj_ppcontrol/follow_traj_ppcontroller.h"

const bool DEBUG=true;

namespace DecisionPure{
    PPControl::PPControl(ros::NodeHandle nh, const std::vector<double> &traj_points)
    {
        trajectory_points = traj_points;
        control_signal_pub  =  nh.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",10);
        current_pose_sub = nh.subscribe("/vesc/odom", 1, &PPControl::CurrentPoseCallback,this);
        ros::param::get("~para_Lfc", PPControl::Lfc);
	    ros::param::get("~para_k", PPControl::k);
	    ROS_INFO("Lfc: %.3f, k: %.3f", PPControl::Lfc, PPControl::k);
        ros::Duration(1).sleep(); //sleep for 1s to wait the sub and pub init

        PPControl::next_pose.x = PPControl::trajectory_points[0];
        PPControl::next_pose.y = PPControl::trajectory_points[1];
    } // PPControl
 
    void PPControl::ControlSignal()
    {
        ros::spinOnce();
        ros::Rate loop_rate(20);

        while(ros::ok())
        {
            GoNextPoseCallback();           
            loop_rate.sleep();
            ros::spinOnce();
        }
    } // ControlSignal

    void PPControl::GoNextPoseCallback()
    { 
        bool hasnextpoint;
        hasnextpoint = FindNextTrajectoryPose();

        ackermann_msgs::AckermannDriveStamped Ackermann_msg;
        ros::Time timestamp = ros::Time::now();
	    
        Ackermann_msg.header.stamp = timestamp;
        Ackermann_msg.header.frame_id = "base_link";
        Ackermann_msg.drive.acceleration = 0;
        Ackermann_msg.drive.jerk = 0;
        Ackermann_msg.drive.steering_angle_velocity = 0;

        PPControl::alpha = atan2(PPControl::next_pose.y-PPControl::current_pose.y, PPControl::next_pose.x-PPControl::current_pose.x) - PPControl::current_pose.theta;
        PPControl::Lf =  PPControl::k*PPControl::current_velocity+ PPControl::Lfc;       
        PPControl::steering_angle_pub = atan2(2*PPControl::L*sin(PPControl::alpha)/PPControl::Lf, 1.0);        
        PPControl::speed_pub = PPControl::trajectory_points[3*PPControl::point_index+2];
        
        if(DEBUG)
        {
            double distan;
            distan = sqrt(pow(PPControl::next_pose.x-PPControl::current_pose.x,2)+pow(PPControl::next_pose.y-PPControl::current_pose.y,2));
            ROS_INFO("Delta y: %.3f",PPControl::next_pose.y-PPControl::current_pose.y);
            ROS_INFO("Delta x: %.3f",PPControl::next_pose.x-PPControl::current_pose.x);
            ROS_INFO("Distance: %.3f", distan);
            ROS_INFO("alpha: %.3f",PPControl::alpha);
            ROS_INFO("Lf: %.3f", PPControl::Lf);        
            ROS_INFO("steering_angle_exact: %.3f", atan2(2*PPControl::L*sin(PPControl::alpha)/distan, 1.0));
            ROS_INFO("Radius: %.3f",PPControl::L*tan(PPControl::steering_angle_pub));
        }

        Ackermann_msg.drive.steering_angle = PPControl::steering_angle_pub;
        Ackermann_msg.drive.speed = PPControl::speed_pub;
        
	if(!hasnextpoint)
        {
	    if(pow(PPControl::current_pose.x-PPControl::next_pose.x,2)+pow(PPControl::current_pose.y-PPControl::next_pose.y,2)<pow(0.2,2)){
            Ackermann_msg.drive.steering_angle = 0;
            Ackermann_msg.drive.speed = 0;
            control_signal_pub.publish(Ackermann_msg);
	        ROS_INFO("Get Final");	
            return;
	    }
        }
        control_signal_pub.publish(Ackermann_msg);
        if(DEBUG) ROS_INFO("steering_angle_publisher: %.3f,speed_publisher:%.3f", PPControl::steering_angle_pub,PPControl::speed_pub);	

    } // GoNextPoseCallback

    bool PPControl::FindNextTrajectoryPose()
    {
        if ( PPControl::trajectory_points.size()<=3*(PPControl::point_index+1))
            return false ;

//        double line_angle = 0.0;
//        double car_point_angle = 0.0;
//        line_angle = atan2(PPControl::trajectory_points[3*(PPControl::point_index+1)+1]-PPControl::trajectory_points[3*PPControl::point_index+1], PPControl::trajectory_points[3*(PPControl::point_index+1)]-PPControl::trajectory_points[3*PPControl::point_index]);
//        car_point_angle = atan2(PPControl::current_pose.y-PPControl::trajectory_points[3*PPControl::point_index+1], PPControl::current_pose.x-PPControl::trajectory_points[3*PPControl::point_index]);

//        if((car_point_angle-line_angle) >-M_PI/2 && (car_point_angle-line_angle)<M_PI/2)
//        {
//            PPControl::point_index++;
//        }
        std::vector<double> distan2(PPControl::trajectory_points.size()/3,0);
        auto distan2_iter = distan2.begin();
        auto distan2_e = distan2.end();
        auto points_iter = PPControl::trajectory_points.cbegin();
        double refpose_theta = 0.0;
        bool iflag = false;
        while(distan2_iter!=distan2_e)
        {
            *distan2_iter =  pow(PPControl::current_pose.x-*points_iter,2)+pow(PPControl::current_pose.y-*(points_iter+1),2);
            points_iter = points_iter + 3;
            distan2_iter++; 
        }

        std::vector<double>::iterator distan2_smallest = std::min_element(std::begin(distan2), std::end(distan2));
        point_index = std::distance(std::begin(distan2), distan2_smallest);
       
        // Check if the refernce pose is lagging behind the current pose
        // If ture, choose the next point
        refpose_theta = atan2( PPControl::trajectory_points[3*PPControl::point_index+1],PPControl::trajectory_points[3*PPControl::point_index]);
        iflag = (cos(refpose_theta)*(PPControl::current_pose.x-PPControl::trajectory_points[3*PPControl::point_index])+sin( refpose_theta)*PPControl::current_pose.y-PPControl::trajectory_points[3*PPControl::point_index+1]) > 0;
        if(iflag)
            PPControl::point_index++;

        PPControl::next_pose.x = PPControl::trajectory_points[3*PPControl::point_index];
        PPControl::next_pose.y = PPControl::trajectory_points[3*PPControl::point_index+1];
        if(DEBUG) ROS_INFO("Points_to_pursuit: %d", PPControl::point_index);	
        return true;

    } // FindNextTrajectoryPose

    void PPControl::CurrentPoseCallback(const nav_msgs::Odometry& odom)
    {
        PPControl::current_pose.x = odom.pose.pose.position.x;
        PPControl::current_pose.y = odom.pose.pose.position.y;
        PPControl::current_pose.theta = 2*acos(odom.pose.pose.orientation.w);
        PPControl::current_velocity =  odom.twist.twist.linear.x;
        if(DEBUG) ROS_INFO("current_pose: %.3f   %.3f   %.3f; current_velocity: %.3f", PPControl::current_pose.x, PPControl::current_pose.y, PPControl::current_pose.theta, PPControl::current_velocity);
    } // CurrentPoseCallback
}
