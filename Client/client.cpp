#include <aris.h>

int sendRequest(int argc, char *argv[], const char *xmlFileName)
{
	std::string cmdName(argv[0]);

	for (int i = 1; i < argc; ++i)
	{
		cmdName = cmdName + " " + argv[i];
	}

    std::cout << "CMD is " <<  cmdName << std::endl;
     
	// 构造msg，这里需要先copy命令名称，然后依次copy各个参数 //
	aris::core::Msg msg;
	msg.copy(cmdName.c_str());

	// 连接并发送msg //
	aris::core::XmlDocument doc;

	if (doc.LoadFile(xmlFileName) != 0)	
        throw std::logic_error("failed to read configuration xml file");

	std::string ip = doc.RootElement()->FirstChildElement("Server")->Attribute("ip");
	std::string port = doc.RootElement()->FirstChildElement("Server")->Attribute("port");

	aris::core::Socket conn;

	while (true)
	{
		try
		{
			conn.connect(ip.c_str(), port.c_str());
			break;
		}
		catch (std::exception &)
		{
			std::cout << "failed to connect server, will retry in 1 second" << std::endl;
			aris::core::msSleep(1000);
		}

	}

	aris::core::Msg ret = conn.sendRequest(msg);

	/*错误处理*/
	if (ret.size() > 0)
	{
		std::cout << "cmd has fault, please regard to following information:" << std::endl;
		std::cout << "    " << ret.data() << std::endl;
	}
	else
	{
		std::cout << "send command successful" << std::endl;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	if (argc <= 1)
        throw std::runtime_error("please input the cmd name");

    // use relative path
    sendRequest(argc - 1, argv + 1, "../../Server/RobotO13.xml");

	return 0;
}
