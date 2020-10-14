#include "follow_traj_skcontrol/follow_traj_skcontroller.h"

const bool DEBUG=true;

namespace DecisionSK{
    SKControl::SKControl(ros::NodeHandle nh, const std::vector<double> &traj_points)
    {
        trajectory_points = traj_points;
        control_signal_pub  =  nh.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",10);
        current_pose_sub = nh.subscribe("/vesc/odom", 1, &SKControl::CurrentPoseCallback,this);
	    ros::param::get("~para_k", SKControl::k);
	    ROS_INFO("k: %.3f", SKControl::k);
        ros::Duration(1).sleep(); //sleep for 1s to wait the sub and pub init

        SKControl::next_pose.x = SKControl::trajectory_points[0];
        SKControl::next_pose.y = SKControl::trajectory_points[1];
    } // SKControl
 
    void SKControl::ControlSignal()
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

    void SKControl::GoNextPoseCallback()
    { 
        bool hasnextpoint;
        hasnextpoint = FindNextTrajectoryPose();
        // Stanley Kinematic Controller uses front pose, transform from rear to front
        SKControl::next_pose.x = SKControl::next_pose.x + SKControl::L*cos(SKControl::next_pose.theta);
        SKControl::next_pose.y = SKControl::next_pose.y + SKControl::L*sin(SKControl::next_pose.theta);

        ackermann_msgs::AckermannDriveStamped Ackermann_msg;
        ros::Time timestamp = ros::Time::now();
	    
        Ackermann_msg.header.stamp = timestamp;
        Ackermann_msg.header.frame_id = "base_link";
        Ackermann_msg.drive.acceleration = 0;
        Ackermann_msg.drive.jerk = 0;
        Ackermann_msg.drive.steering_angle_velocity = 0;

        SKControl::crosstrack_error = (SKControl::next_pose.y-SKControl::current_pose.y)*cos(SKControl::next_pose.theta)-sin(SKControl::next_pose.theta)*(SKControl::next_pose.x-SKControl::current_pose.x);
        SKControl::psi = SKControl::next_pose.theta - SKControl::current_pose.theta;    
    	if (SKControl::current_velocity==0)
            SKControl::steering_angle_pub = 0;
        else
        SKControl::steering_angle_pub = psi + atan2(SKControl::k*SKControl::crosstrack_error/SKControl::current_velocity, 1.0);  
 
        SKControl::speed_pub = SKControl::trajectory_points[4*SKControl::point_index+3];
        
        if(DEBUG)
        {
            double distan;
            distan = sqrt(pow(SKControl::next_pose.x-SKControl::current_pose.x,2)+pow(SKControl::next_pose.y-SKControl::current_pose.y,2));
            ROS_INFO("Delta y: %.3f",SKControl::next_pose.y-SKControl::current_pose.y);
            ROS_INFO("Delta x: %.3f",SKControl::next_pose.x-SKControl::current_pose.x);
            ROS_INFO("Distance: %.3f", distan);
            ROS_INFO("Cross track error: %.3f",SKControl::crosstrack_error);
            ROS_INFO("Psi: %.3f", SKControl::psi);        
        }

        Ackermann_msg.drive.steering_angle = SKControl::steering_angle_pub;
        Ackermann_msg.drive.speed = SKControl::speed_pub;
        
        if(!hasnextpoint)
        {
            if(pow(SKControl::current_pose.x-SKControl::next_pose.x,2)+pow(SKControl::current_pose.y-SKControl::next_pose.y,2)<pow(0.2,2)){
                Ackermann_msg.drive.steering_angle = 0;
                Ackermann_msg.drive.speed = 0;
                control_signal_pub.publish(Ackermann_msg);
                ROS_INFO("Get Final");	
                return;
            }
        }
        control_signal_pub.publish(Ackermann_msg);
        if(DEBUG) ROS_INFO("steering_angle_publisher: %.3f,speed_publisher:%.3f", SKControl::steering_angle_pub,SKControl::speed_pub);	

    } // GoNextPoseCallback

    bool SKControl::FindNextTrajectoryPose()
    {
        if ( SKControl::trajectory_points.size()<=4*(SKControl::point_index+1))
            return false ;

//        double line_angle = 0.0;
//        double car_point_angle = 0.0;
//        line_angle = atan2(SKControl::trajectory_points[3*(SKControl::point_index+1)+1]-SKControl::trajectory_points[3*SKControl::point_index+1], SKControl::trajectory_points[3*(SKControl::point_index+1)]-SKControl::trajectory_points[3*SKControl::point_index]);
//        car_point_angle = atan2(SKControl::current_pose.y-SKControl::trajectory_points[3*SKControl::point_index+1], SKControl::current_pose.x-SKControl::trajectory_points[3*SKControl::point_index]);

//        if((car_point_angle-line_angle) >-M_PI/2 && (car_point_angle-line_angle)<M_PI/2)
//        {
//            SKControl::point_index++;
//        }
        std::vector<double> distan2(SKControl::trajectory_points.size()/4,0);
        auto distan2_iter = distan2.begin();
        auto distan2_e = distan2.end();
        auto points_iter = SKControl::trajectory_points.cbegin();
        double refpose_theta = 0.0;
        bool iflag = false;
        while(distan2_iter!=distan2_e)
        {
            *distan2_iter =  pow(SKControl::current_pose.x-*points_iter,2)+pow(SKControl::current_pose.y-*(points_iter+1),2);
            points_iter = points_iter + 4;
            distan2_iter++; 
        }

        std::vector<double>::iterator distan2_smallest = std::min_element(std::begin(distan2), std::end(distan2));
        point_index = std::distance(std::begin(distan2), distan2_smallest);
       
        // Check if the refernce pose is lagging behind the current pose
        // If ture, choose the next point
        refpose_theta = atan2( SKControl::trajectory_points[4*SKControl::point_index+1],SKControl::trajectory_points[4*SKControl::point_index]);
        iflag = (cos(refpose_theta)*(SKControl::current_pose.x-SKControl::trajectory_points[4*SKControl::point_index])+sin( refpose_theta)*SKControl::current_pose.y-SKControl::trajectory_points[4*SKControl::point_index+1]) > 0;
        if(iflag)
            SKControl::point_index++;

        SKControl::next_pose.x = SKControl::trajectory_points[4*SKControl::point_index];
        SKControl::next_pose.y = SKControl::trajectory_points[4*SKControl::point_index+1];
        SKControl::next_pose.theta = SKControl::trajectory_points[4*SKControl::point_index+2];
        if(DEBUG) ROS_INFO("Points_to_pursuit: %d, x_coordinate: %.3f, y_coordinate: %.3f", SKControl::point_index,SKControl::next_pose.x,SKControl::next_pose.y);
        return true;

    } // FindNextTrajectoryPose

    void SKControl::CurrentPoseCallback(const nav_msgs::Odometry& odom)
    {
        SKControl::current_pose.x = odom.pose.pose.position.x;
        SKControl::current_pose.y = odom.pose.pose.position.y;
        SKControl::current_pose.theta = 2*acos(odom.pose.pose.orientation.w);
        SKControl::current_velocity =  odom.twist.twist.linear.x;
        SKControl::current_pose.x = SKControl::current_pose.x + SKControl::L*cos(SKControl::current_pose.theta);
        SKControl::current_pose.y = SKControl::current_pose.y + SKControl::L*sin(SKControl::current_pose.theta);

        if(DEBUG) ROS_INFO("current_pose: %.3f   %.3f   %.3f; current_velocity: %.3f", SKControl::current_pose.x, SKControl::current_pose.y, SKControl::current_pose.theta, SKControl::current_velocity);
    } // CurrentPoseCallback
}
