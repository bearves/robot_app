/* 步态功能：摇晃身体
* by zhaoyue, 2016-12-12
*/

/*将以下注释代码添加到xml文件*/
/*
            <sw default="sw_param">
                <sw_param type="group">
                    <totalCount abbreviation="t" type="int" default="8000"/>
                    <xAngle abbreviation="x" type="double" default="0"/>
                    <zAngle abbreviation="z" type="double" default="0"/>
                    <yAngle abbreviation="y" type="double" default="0"/>
                    <rDistance abbreviation="r" type="double" default="0.5"/>
                    <yDistance abbreviation="d" type="double" default="-0.05"/>
                </sw_param>
            </sw>
*/

#pragma once
#include <iostream>
#include <cstring>
#include <iomanip>
#include <bitset>
#include <cstring>
#include <map>
#include <string>
#include <stdlib.h>
#include <atomic>

#include <aris.h>
#include <Robot_Gait.h>
#include <Robot_Base.h>

#ifndef PI
#define PI 3.141592653589793
#endif

struct swParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 8000 };
    double rDistance{ 0 };
    double xAngle{ 0 };
    double zAngle{ 0 };
    double yDistance{ 0 };
    double yAngle{ 0 };
};

auto swingParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto swingGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;
