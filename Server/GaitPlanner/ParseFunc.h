#ifndef PARSE_FUNC_H
#define PARSE_FUNC_H

#include <aris.h>
#include "Kinematics/RobotDefinitions.h"

namespace robot_app
{

void motorSelector(const std::map<std::string, std::string> &dom, aris::server::BasicFunctionParam &param)
{
    using namespace kinematics;

    for (auto &i : dom)
    {
        if (i.first == "all")
        {
            std::fill_n(param.active_motor, MOTION_NUM, true);
        }
        else if (i.first == "first")
        {
            std::fill_n(param.active_motor, MOTION_NUM, false);
            for (int j = 0; j < LEG_NUM; j++)
            {
                param.active_motor[j*LEG_DOF] = true;
            }
        }
        else if (i.first == "second")
        {
            std::fill_n(param.active_motor, MOTION_NUM, false);
            for (int j = 0; j < LEG_NUM; j++)
            {
                param.active_motor[j*LEG_DOF+1] = true;
            }
        }
        else if (i.first == "motor")
        {
            int id = { stoi(i.second) };
            if (id<0 || id>=MOTION_NUM)
                throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, MOTION_NUM, false);
            param.active_motor[id] = true;
        }
        else if (i.first == "physical_motor")
        {
            int id = { stoi(i.second) };
            if (id<0 || id>=MOTION_NUM)
                throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, MOTION_NUM, false);
            param.active_motor[aris::server::ControlServer::instance().controller().motionAtPhy(id).absID()] = true;
        }
        else if (i.first == "leg")
        {
            auto leg_id = std::stoi(i.second);
            if (leg_id<0 || leg_id>=LEG_NUM)throw std::runtime_error("invalid param in basicParse func");

            std::fill_n(param.active_motor, MOTION_NUM, false);
            std::fill_n(param.active_motor + leg_id * LEG_DOF, LEG_DOF, true);
        }
    }
}

}
#endif
