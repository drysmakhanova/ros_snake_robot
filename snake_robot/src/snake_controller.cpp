#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>

int main(int argc, char** argv) {
    ros::init(argc, argv, "snake_controller_cpp");
    ros::NodeHandle nh;

    // Create publisher for joint trajectories
    ros::Publisher trajectory_pub = nh.advertise<trajectory_msgs::JointTrajectory>(
        "/snake/follow_joint_trajectory/command", 10
    );

    // Configure joint names (match your URDF)
    std::vector<std::string> joint_names = {
        "Joint1", "Joint2", "Joint3", "Joint4", "Joint5"
    };

    // Wave parameters
    const double amplitude = 0.5;    // radians
    const double frequency = 0.5;    // Hz
    const double phase_shift = (2 * M_PI) / joint_names.size();

    int iteration_count = 0;  // Counter for iterations
    const int max_iterations = 500;  // Stop after 5 iterations
    
    ros::Rate rate(10);  // 10 Hz


    while(ros::ok() && iteration_count < max_iterations) {

		trajectory_msgs::JointTrajectory trajectory;
		trajectory.joint_names = joint_names;

		trajectory_msgs::JointTrajectoryPoint point;
		const double t = ros::Time::now().toSec();

		// Calculate positions using sine wave
		for(size_t i = 0; i < joint_names.size(); i++) {
		    point.positions.push_back(
		        amplitude * std::sin(2 * M_PI * frequency * t + i * phase_shift)
		    );
		}

		// Set time from start (adjust for smooth motion)
		point.time_from_start = ros::Duration(0.1);

		trajectory.points.push_back(point);

		// Publish trajectory
		trajectory_pub.publish(trajectory);
        	iteration_count++;  // Increment counter
		rate.sleep();
	
    }
    trajectory_msgs::JointTrajectory stop_trajectory;
    stop_trajectory.joint_names = joint_names;
    trajectory_msgs::JointTrajectoryPoint stop_point;
    stop_point.positions = std::vector<double>(joint_names.size(), 0.0);
    stop_point.time_from_start = ros::Duration(0.1);
    stop_trajectory.points.push_back(stop_point);
    trajectory_pub.publish(stop_trajectory);

    ROS_INFO("Stopped after %d iterations", max_iterations);

    return 0;
}
