#ifndef PARSE_FUNC_H
#define PARSE_FUNC_H

#include <aris.h>

namespace RobotApp
{


auto basicParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg_out)->void
{
    aris::server::BasicFunctionParam param;

    for (auto &i : params)
    {
        if (i.first == "all")
        {
            std::fill_n(param.active_motor, 13, true);
        }
        else if (i.first == "first")
        {
            std::fill_n(param.active_motor, 13, false);
            for (int j = 0; j < 6; j++)
            {
                param.active_motor[j*2] = true;
            }
        }
        else if (i.first == "second")
        {
            std::fill_n(param.active_motor, 13, false);
            for (int j = 0; j < 6; j++)
            {
                param.active_motor[j*2+1] = true;
            }
        }
        else if (i.first == "motor")
        {
            int id = { stoi(i.second) };
            if (id<0 || id>13)throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, 13, false);
            param.active_motor[id] = true;
        }
        else if (i.first == "physical_motor")
        {
            int id = { stoi(i.second) };
            if (id<0 || id>13)throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, 13, false);
            param.active_motor[aris::server::ControlServer::instance().controller().motionAtPhy(id).absID()] = true;
        }
        else if (i.first == "leg")
        {
            auto leg_id = std::stoi(i.second);
            if (leg_id<0 || leg_id>5)throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, 13, false);
            std::fill_n(param.active_motor + leg_id * 2, 2, true);
        }
    }

    msg_out.copyStruct(param);
}

}
#endif
