/* 步态功能：腿部机构标定
* by zhaoyue, 2018-12-7
*/

/*将以下注释代码添加到xml文件*/
/*
            <cl default="cl_param">
                <cl_param type="group">
                    <totalCount abbreviation="t" type="int" default="2000"/>
                    <leg abbreviation="l" type="int" default="0"/>
                    <cmd_param type="unique" default="none">
                        <next abbreviation="n"/>
                        <previous abbreviation="p"/>
                        <quit abbreviation="q"/>
                    </cmd_param>
                </cl_param>
            </cl>
*/

#ifndef CALIBRATION_H
#define CALIBRATION_H

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

enum class Command
{
    NONE = 0,
    NEXT = 1,
    PREVIOUS = 2,
    QUIT = 99
};

class CaliCmd
{
public:
    static CaliCmd& getCmd()
    {
        static CaliCmd c;
        return c;
    }
    Command& cmd() { return cmd_; }
private:
    Command cmd_{ Command::NEXT };
    CaliCmd() = default;
};

/*gait parameters*/
struct caliParam final :public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 8000 };
    int leg_id{ 0 };
};

auto calibrationParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto calibrationGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // CALIBRATION_H
