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
#include "GaitPlanner/WalkPlanner.h"
#include "GaitPlanner/StairClimbPlanner.h"
#include "GaitPlanner/HillClimbPlanner.h"
#include "GaitPlanner/FastWalk.h"
#include "GaitPlanner/HelloPlanner.h" 
#include "GaitPlanner/HeightAdjustment.h"


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
    robot_app::WalkPlanner::setMotionSelector(robot_app::motorSelector);
    robot_app::StairClimbPlanner::setMotionSelector(robot_app::motorSelector);
    robot_app::HillClimbPlanner::setMotionSelector(robot_app::motorSelector);
    robot_app::FastWalk::setMotionSelector(robot_app::motorSelector);
    robot_app::HelloPlanner::setMotionSelector(robot_app::motorSelector);
    robot_app::HeightAdjustment::setMotionSelector(robot_app::motorSelector);
    
    auto &rs = aris::server::ControlServer::instance();
    
    rs.createModel<aris::model::Model>();
    rs.loadXml(xml_address.c_str());
    rs.setMotionSelector(robot_app::motorSelector);
    rs.addCmd("rc", robot_app::RecoverPlanner::recoverParser, robot_app::RecoverPlanner::recover);
    rs.addCmd("wk", robot_app::WalkPlanner::walkParser, robot_app::WalkPlanner::walk);
    rs.addCmd("sc",robot_app::StairClimbPlanner::stairclimbParser,robot_app::StairClimbPlanner::stairclimb);
    rs.addCmd("hc",robot_app::HillClimbPlanner::hillclimbParser,robot_app::HillClimbPlanner::hillclimb);
    rs.addCmd("fwk",robot_app::FastWalk::fastWalkParser, robot_app::FastWalk::fastWalk);
    rs.addCmd("hello", robot_app::HelloPlanner::helloParser, robot_app::HelloPlanner::hello);
    rs.addCmd("height", robot_app::HeightAdjustment::heightParser, robot_app::HeightAdjustment::height);
    rs.open();
    
    rs.setOnExit([&]() 
    {
    	aris::core::stopMsgLoop();
    });
    aris::core::runMsgLoop();
    
    return 0;
}
