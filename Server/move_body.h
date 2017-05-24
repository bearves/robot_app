/* 步态功能：移动身体
 * by liujimu, 2016-12-12
 */

/*将以下注释代码添加到xml文件*/
/*
            <mb default="mb_param">
                <mb_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <x abbreviation="x" type="double" default="0"/>
                    <y abbreviation="y" type="double" default="0"/>
                    <z abbreviation="z" type="double" default="0"/>
                    <pitch abbreviation="u" type="double" default="0"/>
                    <yaw abbreviation="v" type="double" default="0"/>
                    <roll abbreviation="w" type="double" default="0"/>
                </mb_param>
            </mb>
*/

#ifndef MOVE_BODY_H
#define MOVE_BODY_H

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

struct mbParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
    double x{ 0 };
    double y{ 0 };
    double z{ 0 };
	double pitch{ 0 };
	double yaw{ 0 };
	double roll{ 0 };
};

auto moveBodyParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto moveBodyGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // MOVE_BODY_H
