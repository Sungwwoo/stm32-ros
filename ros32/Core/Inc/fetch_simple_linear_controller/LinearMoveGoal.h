#ifndef _ROS_fetch_simple_linear_controller_LinearMoveGoal_h
#define _ROS_fetch_simple_linear_controller_LinearMoveGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PointStamped.h"

namespace fetch_simple_linear_controller
{

  class LinearMoveGoal : public ros::Msg
  {
    public:
      typedef geometry_msgs::PointStamped _point_type;
      _point_type point;
      typedef bool _hold_final_pose_type;
      _hold_final_pose_type hold_final_pose;

    LinearMoveGoal():
      point(),
      hold_final_pose(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->point.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_hold_final_pose;
      u_hold_final_pose.real = this->hold_final_pose;
      *(outbuffer + offset + 0) = (u_hold_final_pose.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->hold_final_pose);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->point.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_hold_final_pose;
      u_hold_final_pose.base = 0;
      u_hold_final_pose.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->hold_final_pose = u_hold_final_pose.real;
      offset += sizeof(this->hold_final_pose);
     return offset;
    }

    const char * getType(){ return "fetch_simple_linear_controller/LinearMoveGoal"; };
    const char * getMD5(){ return "20b4a125083c890036de2d9ef7e007c0"; };

  };

}
#endif
