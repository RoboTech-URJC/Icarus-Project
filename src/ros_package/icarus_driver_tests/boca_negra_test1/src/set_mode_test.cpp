/* Author: Fernando González fergonzaramos@yahoo.es  */

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "boca_negra/Bocanegra.hpp"

#define HZ 2

class StatesMachine : public boca_negra::Bocanegra
{
public:

	StatesMachine()
	: nh_("~"), state_(SETTING_MODE)
	{
		mode_pub_ = nh_.advertise<std_msgs::String>("/icarus_driver/set_mode", 1);
		setting_mode_ts = ros::Time::now();
	}

	bool
	update()
	{
		std_msgs::String mode;

		if (state_ == SETTING_MODE) {
			activate("set_mode_node");
			mode.data = "AUTO.TAKEOFF";
			mode_pub_.publish(mode);

			if ((ros::Time::now() - setting_mode_ts).toSec() >= 1.0)
				state_ = FINISH;
		} else if(state_ == FINISH) {
			deactivate("set_mode_node");
		}

		return state_ == FINISH;
	}

private:

	static const int SETTING_MODE = 0;
	static const int FINISH = 1;

	ros::NodeHandle nh_;

	ros::Time setting_mode_ts;
	int state_;

	ros::Publisher mode_pub_;
};

int
main(int argc, char **argv)
{
	ros::init(argc, argv, "set_mode_test_node");

	StatesMachine hfsm;

	ros::Rate loop_rate(HZ);
	bool finish = false;
	while (ros::ok()) {
		finish = hfsm.update();
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	return 0;
}
