#include <iostream>
#include <cstring>
#include <iomanip> 
#include <bitset>
#include <cstring>
#include <map>
#include <string>

using namespace std;

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Type_I.h>

#include "swing.h"
#include "move_body.h"
#include "twist_waist.h"
#include "say_hello.h"

#ifdef WIN32
#define rt_printf printf
#endif
#ifdef UNIX
#include "rtdk.h"
#include "unistd.h"
#endif


int main(int argc, char *argv[])
{
	std::string xml_address;

	if (std::string(argv[1]) == "EDU6")
	{
		xml_address = "../../Server/RobotEDU6.xml";
	}
	else
	{
		throw std::runtime_error("invalid robot name, please type in EDU6");
	}
	
	auto &rs = aris::server::ControlServer::instance();
	

	rs.createModel<Robots::RobotTypeI>();
	rs.loadXml(xml_address.c_str());
	rs.addCmd("en", Robots::basicParse, nullptr);
	rs.addCmd("ds", Robots::basicParse, nullptr);
	rs.addCmd("hm", Robots::basicParse, nullptr);
	rs.addCmd("rc", Robots::recoverParse, Robots::recoverGait);
	rs.addCmd("wk", Robots::walkParse, Robots::walkGait);
	rs.addCmd("ro", Robots::resetOriginParse, Robots::resetOriginGait);
	rs.addCmd("mb", moveBodyParse, moveBodyGait);
	rs.addCmd("sw", swingParse, swingGait);
	rs.addCmd("tw", twistWaistParse, twistWaistGait);
	rs.addCmd("sh", sayHelloParse, sayHelloGait);

	rs.open();

	rs.setOnExit([&]() 
	{
		aris::core::XmlDocument xml_doc;
		xml_doc.LoadFile(xml_address.c_str());
		auto model_xml_ele = xml_doc.RootElement()->FirstChildElement("Model");
		if (!model_xml_ele)throw std::runtime_error("can't find Model element in xml file");
		rs.model().saveXml(*model_xml_ele);
		
		aris::core::stopMsgLoop();
	});
	aris::core::runMsgLoop();
	
	return 0;
}
