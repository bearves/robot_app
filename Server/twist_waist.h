/* 步态功能：扭腰
 * by liujimu, 2016-4-1
 */

/*将以下注释代码添加到xml文件*/
/*
            <tw default="tw_param">
                <tw_param type="group">
                    <totalCount abbreviation="t" type="int" default="12000"/>
                    <pitchMax abbreviation="p" type="double" default="15"/>
                    <rollMax abbreviation="r" type="double" default="15"/>
                    <diameter abbreviation="d" type="double" default="0.05"/>
                    <height abbreviation="h" type="double" default="0"/>
                </tw_param>
            </tw>
*/

#ifndef TWIST_WAIST_H
#define TWIST_WAIST_H

#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct twParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 12000 };
    double pitchMax{ PI * 15 / 180 };
    double rollMax{ PI * 15 / 180 };
    double diameter{ 0.05 };
    double height{ 0 };
};

auto twistWaistParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto twistWaistGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // TWIST_WAIST_H
