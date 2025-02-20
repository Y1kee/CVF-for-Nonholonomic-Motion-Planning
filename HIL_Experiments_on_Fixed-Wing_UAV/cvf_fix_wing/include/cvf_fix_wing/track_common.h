/*
 * @Author: “Shirley” “todo@163.com”
 * @Date: 2023-05-04 15:27:54
 * @LastEditors: “Shirley” shirleycoding@163.com
 * @LastEditTime: 2024-11-06 09:32:32
 * @FilePath: /src/planner_tracker/track_controller/include/track_controller/track_common.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __TRACK_COMMON_H
#define __TRACK_COMMON_H

#include <string>
#define FLIGHT_TYPE 1

#define DBG 1

namespace coopfly
{
    enum CONTROL_MODE   
    {
        CIRCLE,
        OFFLINE_TRAJ_TRACK,
        REALTIME_TRAJ_TRACK
    };
    const std::string control_mode_str[4] = {"CIRCLE", "OFFLINE_TRAJ_TRACK", "REALTIME_TRAJ_TRACK"};

    enum TRACK_STATE
    {
        INIT,
        CIRCLE_WAITING,
        TRACKING
    };
    const std::string track_state_str[4] = {"INIT", "CIRCLE_WAITING", "TRACKING"};

    enum TRAJ_EXE_TYPE
    {
        SUCCESSIVE,
        IMMEDIATE,
        FINISHED
    };
    const std::string traj_exe_type_str[3] = {"SUCCESSIVE", "IMMEDIATE", "FINISHED"};

    enum CALL_REPLAN_STATE
    {
        TO_CALL,   // executing trajectory
        CALLING,   // calling for new trajectory
        CALLED,    // have called for this trajectory
        CALLING_G, // calling for new trajectory for the first time
        CALL_SLEEP
    };
    const std::string call_replan_state_str[5] =
        {"TO_CALL", "CALLING", "CALLED", "CALLING_G", "CALL_SLEEP"};

} // namespace coopfly
#endif