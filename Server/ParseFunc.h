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
            std::fill_n(param.active_motor, 18, true);
        }
        else if (i.first == "first")
        {
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 0, 3, true);
            std::fill_n(param.active_motor + 6, 3, true);
            std::fill_n(param.active_motor + 12, 3, true);
        }
        else if (i.first == "second")
        {
            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + 3, 3, true);
            std::fill_n(param.active_motor + 9, 3, true);
            std::fill_n(param.active_motor + 15, 3, true);
        }
        else if (i.first == "motor")
        {
            int id = { stoi(i.second) };
            if (id<0 || id>17)throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, 18, false);
            param.active_motor[id] = true;
        }
        else if (i.first == "physical_motor")
        {
            int id = { stoi(i.second) };
            if (id<0 || id>5)throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, 18, false);
            param.active_motor[aris::server::ControlServer::instance().controller().motionAtPhy(id).absID()] = true;
        }
        else if (i.first == "leg")
        {
            auto leg_id = std::stoi(i.second);
            if (leg_id<0 || leg_id>5)throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, 18, false);
            std::fill_n(param.active_motor + leg_id * 3, 3, true);
        }
    }

    msg_out.copyStruct(param);
}

}
#endif
