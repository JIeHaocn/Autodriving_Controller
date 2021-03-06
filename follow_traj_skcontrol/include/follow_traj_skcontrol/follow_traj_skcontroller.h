
#include <ros/ros.h>
#include <vector>
#include <cmath>
#include <geometry_msgs/Pose2D.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <nav_msgs/Odometry.h>

namespace DecisionSK{
    class SKControl{
        public:
            SKControl(ros::NodeHandle nh, const std::vector<double> &traj_points);
            void ControlSignal();

        private:
            int point_index = 0;
            double L = 0.256;
            double k = 1.68;
            double crosstrack_error=0.0;
            double psi = 0.0;
            double current_velocity = 0.0;
            double speed_pub = 0.0;
            double steering_angle_pub = 0.0;
            std::vector<double> trajectory_points;
            ros::Publisher  control_signal_pub;
            ros::Subscriber trajectory_next_pose_sub;
            ros::Subscriber current_pose_sub;
            geometry_msgs::Pose2D current_pose, next_pose;

            bool FindNextTrajectoryPose();
            void CurrentPoseCallback(const nav_msgs::Odometry& odom);
            void GoNextPoseCallback();
    }; // class SKControl
}
