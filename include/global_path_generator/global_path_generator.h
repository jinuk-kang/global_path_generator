#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>

#define FILE_NAME "/home/path.txt"
#define LOOP_RATE 10 // use it for sleep time to determine the distance between points in the path
#define INIT_DIST_HP 0.00001

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
		double o_x;
		double o_y;
		double o_z;

	public:
		OdomDouble() {}

		OdomDouble(double x, double y, double z, double o_x, double o_y, double o_z) {
			this->x = x;
			this->y = y;
			this->z = z;
			this->o_x = o_x;
			this->o_y = o_y;
			this->o_z = o_z;
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

		void setOX(double o_x) {
			this->o_x = o_x;
		}

		void setOY(double o_y) {
			this->o_y = o_y;
		}

		void setOZ(double o_z) {
			this->o_z = o_z;
		}

		// getter
		double getX() {
			return this->x;
		}

		double getY() {
			return this->y;
		}

		double getZ() {
			return this->z;
		}

		double getOX() {
			return this->o_x;
		}

		double getOY() {
			return this->o_y;
		}

		double getOZ() {
			return this->o_z;
		}
};

class GlobalPathGenerator {
	private:
        vector<OdomDouble> path_;

    public:
		ros::NodeHandle nh_;
        ros::Subscriber odom_sub_;

		GlobalPathGenerator();

        bool stop_flag_;
        bool clear_flag_;
        double dist_hp_;

        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void printOdom(OdomDouble odomDouble);
		void clearPath();
		OdomDouble getLastOdom();
        void savePath();
};
