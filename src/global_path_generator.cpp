#include "global_path_generator/global_path_generator.h"

// initialize subscriber and parameters
GlobalPathGenerator::GlobalPathGenerator() {
    odom_sub_ = nh_.subscribe("/odom", 1, &GlobalPathGenerator::odomCallback, this);

    // params
    nh_.setParam("/stop_flag", false);
    nh_.setParam("/clear_flag", false);
    nh_.setParam("/dist_hp", INIT_DIST_HP);

    stop_flag_ = false;
    clear_flag_ = false;
    dist_hp_ = INIT_DIST_HP;
}

// get odometry data from hdl_localization node
void GlobalPathGenerator::odomCallback(const nav_msgs::Odometry::ConstPtr &odom) {

    nh_.getParam("/stop_flag", stop_flag_);

	OdomDouble odomDouble(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);

    if (!stop_flag_) {
        printOdom(odomDouble);

        if (!this->path_.empty()) {
            double x = odomDouble.getX();
            double y = odomDouble.getY();

            double last_x = getLastOdom().getX();
            double last_y = getLastOdom().getY();

            double dist = sqrt(pow((last_x-x), 2) + pow(last_y-y, 2));

            // ignore if the distance between points is too close
            if (dist < dist_hp_) {
                return;
            }
        }

		this->path_.push_back(odomDouble);
    }
}

void GlobalPathGenerator::printOdom(OdomDouble odomDouble) { 
	cout << "x : " << odomDouble.getX() << endl;
	cout << "y : " << odomDouble.getY() << endl;
	cout << "z : " << odomDouble.getZ() << endl;
	cout << "o_x : " << odomDouble.getOX() << endl;
	cout << "o_y : " << odomDouble.getOY() << endl;
	cout << "o_z : " << odomDouble.getOZ() << endl;
	cout << "o_w : " << odomDouble.getOW() << endl;
	cout << endl;

    //ROS_INFO("< %f, %f, %f >", odomDouble.getX(), odomDouble.getY(), odomDouble.getZ());
}

OdomDouble GlobalPathGenerator::getLastOdom() {
	return this->path_.back();
}

void GlobalPathGenerator::clearPath() {
	this->path_.clear();
}

// save the path using odometry data
void GlobalPathGenerator::savePath() {

	if (access(FILE_NAME, 0) == 0) {
		int result = remove(FILE_NAME);
	}

    cout << "start saving path..." << endl;
    
    ofstream file(FILE_NAME);

    for (auto odom : this->path_) {
        double x = odom.getX();
        double y = odom.getY();
        double z = odom.getZ();
        double o_x = odom.getOX();
        double o_y = odom.getOY();
        double o_z = odom.getOZ();
        double o_w = odom.getOW();

        if (file.is_open()) {
            string row = to_string(x) + "," + to_string(y) + "," + to_string(z) + "," + to_string(o_x) + "," + to_string(o_y) + "," + to_string(o_z) + "," + to_string(o_w) + "\n";
            file << row;
        }
    }

    file.close();

	cout << endl;
    cout << "--- saved path successfully ---" << endl;
    cout << "text file path -> " << FILE_NAME << endl;
    cout << "path size -> " << to_string(this->path_.size()) << endl;
    cout << "-------------------------------" << endl;
	cout << endl;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "global_path_generator_node");
    GlobalPathGenerator globalPathGenerator;

    ros::Rate loop_rate(LOOP_RATE);

    while(ros::ok()) {
        globalPathGenerator.nh_.getParam("/stop_flag", globalPathGenerator.stop_flag_);
        globalPathGenerator.nh_.getParam("/clear_flag", globalPathGenerator.clear_flag_);

        if (globalPathGenerator.clear_flag_) {
            globalPathGenerator.clearPath();
            globalPathGenerator.nh_.setParam("/clear_flag", false);
            globalPathGenerator.clear_flag_ = false;
        }

        if (globalPathGenerator.stop_flag_) {
			cout << endl;
            cout << "odomCallback stopped" << endl;
            globalPathGenerator.savePath();
            cout << "node terminated" << endl;
			cout << endl;
            ros::shutdown();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
