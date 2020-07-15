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

class GlobalPathGenerator {
    private:
        ros::Subscriber odom_sub_;
        vector<nav_msgs::Odometry> path_;

        bool stop_flag_;
        bool clear_flag_;
        double dist_hp_;

    public:
        void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);
        void printOdom(const nav_msgs::Odometry::ConstPtr& odom);
        void savePath();
};