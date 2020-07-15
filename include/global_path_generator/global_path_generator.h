#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

#define FILE_NAME "path.txt"
#define LOOP_RATE 10 // use it for sleep time to determine the distance between points in the path
#define INIT_DIST_HP 0.5

using namespace std;

// ----------
// Parameters 
// ----------
// stop_flag -> shut down the node
// clear_flag -> clear the path vector
// dist_hp -> determine the distance between points in the path
//

class OdomDouble {
	private:
		double x;
		double y;
		double z;
	public:
		OdomDouble() {}

		OdomDouble(double x, double y, double z) {
			this->x = x;
			this->y = y;
			this->z = z;
		}

		// setter
		void setX(double x) {
			this->x = x;
		}

		void setY(double y) {
			this->y = y;
		}

		void setZ(double z) {
			this->z = z;
		}

		// getter
		double getX() {
			return this->x;
		}

		double getY() {
			return this->x;
		}

		double getZ() {
			return this->x;
		}
};

class GlobalPathGenerator {

    public:
		ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;
        vector<OdomDouble> path_;

		GlobalPathGenerator();

        bool stop_flag_;
        bool clear_flag_;
        double dist_hp_;

        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void printOdom(OdomDouble odomDouble);
        void savePath();
};
