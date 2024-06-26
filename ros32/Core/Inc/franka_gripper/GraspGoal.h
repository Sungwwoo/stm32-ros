#ifndef _ROS_franka_gripper_GraspGoal_h
#define _ROS_franka_gripper_GraspGoal_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "franka_gripper/GraspEpsilon.h"

namespace franka_gripper
{

  class GraspGoal : public ros::Msg
  {
    public:
      typedef double _width_type;
      _width_type width;
      typedef franka_gripper::GraspEpsilon _epsilon_type;
      _epsilon_type epsilon;
      typedef double _speed_type;
      _speed_type speed;
      typedef double _force_type;
      _force_type force;

    GraspGoal():
      width(0),
      epsilon(),
      speed(0),
      force(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_width.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_width.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_width.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_width.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->width);
      offset += this->epsilon.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.real = this->speed;
      *(outbuffer + offset + 0) = (u_speed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_speed.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_speed.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_speed.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_speed.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->speed);
      union {
        double real;
        uint64_t base;
      } u_force;
      u_force.real = this->force;
      *(outbuffer + offset + 0) = (u_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_force.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_force.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_force.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_force.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_force.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->force);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        double real;
        uint64_t base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_width.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->width = u_width.real;
      offset += sizeof(this->width);
      offset += this->epsilon.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_speed;
      u_speed.base = 0;
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_speed.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->speed = u_speed.real;
      offset += sizeof(this->speed);
      union {
        double real;
        uint64_t base;
      } u_force;
      u_force.base = 0;
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_force.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->force = u_force.real;
      offset += sizeof(this->force);
     return offset;
    }

    const char * getType(){ return "franka_gripper/GraspGoal"; };
    const char * getMD5(){ return "627a0f0b10ad0c919fbd62b0b3427e63"; };

  };

}
#endif
