/* 步态功能：寻找各支链U副中心，用于标定
 * by liujimu, 2018-12-11
 */

/*将以下注释代码添加到xml文件*/
/*
            <fjc default="tw_param">
                <fjc_param type="group">
                    <totalCount abbreviation="t" type="int" default="4000"/>
                    <leg abbreviation="l" type="int" default="0"/>
                    <chain abbreviation="c" type="int" default="0"/>
                    <distance abbreviation="d" type="double" default="0.07"/>
                </fjc_param>
            </fjc>
*/

#ifndef FIND_JOINT_CENTER_H
#define FIND_JOINT_CENTER_H

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

struct fjcParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 2000 };
    int leg_id{ 0 };
    int chain_id{ 0 };
    double distance{ 0.08 };
};

auto findJointCenterParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto findJointCenterGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // FIND_JOINT_CENTER_H
