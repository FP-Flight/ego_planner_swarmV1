#ifndef _FSM_MOVE_TO_
#define _FSM_MOVE_TO_
#include <ros/ros.h>
class FSM_MOVE_TO
{
public:
    enum State_t
    {
        INIT = 0,
        TAKEOFF_DONE,
        MOVING,
        // MOVE_TO_POINT_2,
        // MOVE_TO_POINT_3,
    };

    State_t now_state = INIT;
    State_t last_state = INIT;
    void Init_FSM()
    {
        now_state = INIT;
        last_state = INIT;

        take_off_done_flag_ = false;
        move_to_point_1_flag_ = false;
        move_to_point_2_flag_ = false;
        move_to_point_3_flag_ = false;
    }
    inline void process()
    {
        last_state = now_state;
        switch (now_state)
        {
        case INIT:
        {
            if (get_takeoff_done_flag())
            {
                now_state = TAKEOFF_DONE;
            }
            break;
        }

        case TAKEOFF_DONE:
        {
            if (get_move_to_point_1_flag())
            {
                now_state = MOVE_TO_POINT_1;
            }
            break;
        }

        case MOVE_TO_POINT_1:
        {
            if (get_move_to_point_2_flag())
            {
                now_state = MOVE_TO_POINT_2;
            }
            break;
        }

        case MOVE_TO_POINT_2:
        {
            if (get_move_to_point_3_flag())
            {
                now_state = MOVE_TO_POINT_3;
            }
            break;
        }
        }
    }
    // set and get take off flag
    inline void set_takeoff_done_flag(bool flag)
    {
        take_off_done_flag_ = flag;
    }
    inline bool get_takeoff_done_flag()
    {
        return take_off_done_flag_;
    }

    // set and get move to point 1 flag
    inline void set_move_to_point_1_flag(bool flag)
    {
        move_to_point_1_flag_ = flag;
    }
    inline bool get_move_to_point_1_flag()
    {
        return move_to_point_1_flag_;
    }

    // set and get move to point 2 flag
    inline void set_move_to_point_2_flag(bool flag)
    {
        move_to_point_2_flag_ = flag;
    }
    inline bool get_move_to_point_2_flag()
    {
        return move_to_point_2_flag_;
    }

    // set and get move to point 3 flag
    inline void set_move_to_point_3_flag(bool flag)
    {
        move_to_point_3_flag_ = flag;
    }
    inline bool get_move_to_point_3_flag()
    {
        return move_to_point_3_flag_;
    }

    // // set and get ALL take off done
    // inline void set_all_takeoff_done_flag(bool flag)
    // {
    //     all_take_off_done_flag_ = flag;
    // }
    // inline bool get_all_takeoff_done_flag()
    // {
    //     return all_take_off_done_flag_;
    // }

    // // set and get ALL move to point 1 flag
    // inline void set_all_move_to_point_1_flag(bool flag)
    // {
    //     all_move_to_points_1_flag_ = flag;
    // }
    // inline bool get_all_move_to_point_1_flag()
    // {
    //     return all_move_to_points_1_flag_;
    // }

private:
    bool take_off_done_flag_;
    bool move_to_point_1_flag_;
    bool move_to_point_2_flag_;
    bool move_to_point_3_flag_;

    // bool all_take_off_done_flag_;
    // bool all_move_to_points_1_flag_;
    // bool all_move_to_points_2_flag_;
    // bool all_move_to_points_3_flag_;
};

#endif