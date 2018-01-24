#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <aris.h>

#include "rtdk.h"
#include "unistd.h"
#include "GaitPlanner/MotorSelector.h"
#include "GaitPlanner/RecoverPlanner.h"


int main(int argc, char *argv[])
{
    using namespace std;
	std::string xml_address;

	if (argc <= 1)
	{
		std::cout << "Specify a robot name" << std::endl;
        exit(0);
	}
	else if (std::string(argv[1]) == "O13")
	{
		xml_address = "../../Server/RobotO13.xml";
	}
	else
	{
		throw std::runtime_error("invalid robot name");
	}

    robot_app::RecoverPlanner::setMotionSelector(robot_app::motorSelector);

	auto &rs = aris::server::ControlServer::instance();
	
	rs.createModel<aris::model::Model>();
	rs.loadXml(xml_address.c_str());
	rs.setMotionSelector(robot_app::motorSelector);
	rs.addCmd("rc", robot_app::RecoverPlanner::recoverParser, robot_app::RecoverPlanner::recover);

	rs.open();

	rs.setOnExit([&]() 
	{
		aris::core::stopMsgLoop();
	});
	aris::core::runMsgLoop();
	
	return 0;
}
