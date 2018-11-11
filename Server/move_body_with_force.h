/* 步态功能：通过三点触碰找到孔中心，进行插孔
 * by liujimu, 2016-4-26
 */

/*将以下注释代码添加到xml文件*/
/*
            <mbf default="mbf_param">
                <mbf_param type="group">
                    <totalCount abbreviation="t" type="int" default="1500"/>
                    <h abbreviation="h" type="double" default="0.04"/>
                    <offset abbreviation="o" type="double" default="0.02"/>
                </mbf_param>
            </mbf>
            <mbfs/>
*/

#ifndef PEG_IN_HOLE_H
#define PEG_IN_HOLE_H

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

enum class PegInHoleProcess
{
    PREPARE = 0,
    ALIGN = 1,
    INSERT = 2,
    RETURN = 3,
    STOP = 4,
};

class MbfState
{
public:
    static MbfState& getState()
    {
        static MbfState s;
        return s;
    }
    bool& isStopping() { return isStopping_; }
private:
    bool isStopping_{ true };
    MbfState() = default;
};


/*gait parameters*/
struct mbfParam final:public aris::server::GaitParamBase
{
    std::int32_t totalCount{ 1500 };
    double h{ 0.04 };
    double offset{ 0.02 };
};

/*parse function*/
auto moveBodyWithForceParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;
auto moveBodyWithForceStopParse(const std::string &cmd, const std::map<std::string, std::string> &params, aris::core::Msg &msg)->void;

/*operation function*/
auto moveBodyWithForceGait(aris::dynamic::Model &model, const aris::dynamic::PlanParamBase &param_in)->int;

#endif // PEG_IN_HOLE_H
