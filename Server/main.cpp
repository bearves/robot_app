#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>

#include "rtdk.h"
#include "unistd.h"
#include "ParseFunc.h"


int main(int argc, char *argv[])
{
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
	
	auto &rs = aris::server::ControlServer::instance();
	

	rs.createModel<aris::model::Model>();
	rs.loadXml(xml_address.c_str());
	rs.addCmd("en", RobotApp::basicParse, nullptr);
	rs.addCmd("ds", RobotApp::basicParse, nullptr);
	rs.addCmd("hm", RobotApp::basicParse, nullptr);
        rs.addCmd("jog", RobotApp::jogParse, nullptr);

	rs.open();

	rs.setOnExit([&]() 
	{
		aris::core::stopMsgLoop();
	});
	aris::core::runMsgLoop();
	
	return 0;
}
