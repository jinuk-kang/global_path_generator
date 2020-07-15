#include "global_path_generator/global_path_generator.h"

// initialize subscriber and parameters
GlobalPathGenerator::GlobalPathGenerator() {
    ros::NodeHandle nh_;
    odom_sub_ = nh_.subcribe("/odom", 1, &GlobalPathGenerator::odomCallback, this);

    // params
    nh_.setParam("/stop_flag", false);
    nh_.setParam("/clear_flag", false);
    nh_.setParam("/dist_hp", INIT_DIST_HP);

    stop_flag_ = false;
    clear_flag_ = false;
    dist_hp_ = INIT_DIST_HP;
}

// get odometry data from hdl_localization node
void GlobalPathGenerator:::odomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

    nh_.getParam("/stop_flag", stop_flag_);

    if (!stop_flag_) {
        printOdom(odom);

        if (!path_.empty()) {
            double x = odom->pose.pose.position.x;
            double y = odom->pose.pose.position.y;

            double last_x = path_.back()->pose.pose.position.x;
            double last_y = path_.back()->pose.pose.position.y;

            double dist = sqrt(pow((last_x-x), 2) + pow(last_y-y, 2));

            // ignore if the distance between points is too close
            if (dist < dist_hp_) {
                return;
            }
        }

        path_.push_back(odom);
    }
}

void GlobalPathGenerator::printOdom(const nav_msgs::Odometry::ConstPtr& odom) { 
    double x = odom->pose.pose.position.x;
    double y = odom->pose.pose.position.y;
    double z = odom->pose.pose.position.z;

    ROS_INFO("< %f, %f, %f >", x, y, z);
}

// save the path using odometry data
void GlobalPathGenerator::savePath() {
    cout << "start saving path" << endl;
    
    ofstream file(FILE_NAME);

    for (auto odom : path_) {
        double x = odom->pose.pose.position.x;
        double y = odom->pose.pose.position.y;
        double z = odom->pose.pose.position.z;

        if (file.is_open()) {
            string row = to_string(x) + "," + to_string(y) + "," + to_string(z) + "\n";
            file << row;
        }
    }

    file.close();

    cout << "--- saved path successfully ---" << endl;
    cout << "text file path -> " << "./" << FILE_NAME << endl;
    cout << "path size -> " << to_string(path_.size()) << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_path_generator_node");
    GlobalPathGenerator globalPathGenerator;

    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok()) {
        nh_.getParam("/stop_flag", stop_flag_);
        nh_.getParam("/clear_flag", clear_flag_);

        if (clear_flag_) {
            path_.clear();
            nh_.setParam("/clear_flag", false);
            clear_flag_ = false;
        }

        if (stop_flag_) {
            cout << "odomCallback stopped" << endl;
            savePath();
            cout << "node terminated" << endl;
            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}