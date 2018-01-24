#ifndef MOTOR_SELECTOR_H
#define MOTOR_SELECTOR_H

#include <aris.h>
#include "Kinematics/RobotDefinitions.h"

namespace robot_app
{

    void motorSelector(const std::map<std::string, std::string> &dom, aris::server::BasicFunctionParam &param);

}
#endif
