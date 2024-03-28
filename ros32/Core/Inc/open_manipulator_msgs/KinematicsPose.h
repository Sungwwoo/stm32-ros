#ifndef _ROS_open_manipulator_msgs_KinematicsPose_h
#define _ROS_open_manipulator_msgs_KinematicsPose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace open_manipulator_msgs
{

  class KinematicsPose : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _pose_type;
      _pose_type pose;
      typedef double _max_accelerations_scaling_factor_type;
      _max_accelerations_scaling_factor_type max_accelerations_scaling_factor;
      typedef double _max_velocity_scaling_factor_type;
      _max_velocity_scaling_factor_type max_velocity_scaling_factor;
      typedef double _tolerance_type;
      _tolerance_type tolerance;

    KinematicsPose():
      pose(),
      max_accelerations_scaling_factor(0),
      max_velocity_scaling_factor(0),
      tolerance(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_max_accelerations_scaling_factor;
      u_max_accelerations_scaling_factor.real = this->max_accelerations_scaling_factor;
      *(outbuffer + offset + 0) = (u_max_accelerations_scaling_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_accelerations_scaling_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_accelerations_scaling_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_accelerations_scaling_factor.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_accelerations_scaling_factor.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_accelerations_scaling_factor.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_accelerations_scaling_factor.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_accelerations_scaling_factor.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_accelerations_scaling_factor);
      union {
        double real;
        uint64_t base;
      } u_max_velocity_scaling_factor;
      u_max_velocity_scaling_factor.real = this->max_velocity_scaling_factor;
      *(outbuffer + offset + 0) = (u_max_velocity_scaling_factor.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_velocity_scaling_factor.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_velocity_scaling_factor.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_velocity_scaling_factor.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_max_velocity_scaling_factor.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_max_velocity_scaling_factor.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_max_velocity_scaling_factor.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_max_velocity_scaling_factor.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->max_velocity_scaling_factor);
      union {
        double real;
        uint64_t base;
      } u_tolerance;
      u_tolerance.real = this->tolerance;
      *(outbuffer + offset + 0) = (u_tolerance.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tolerance.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tolerance.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tolerance.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_tolerance.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_tolerance.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_tolerance.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_tolerance.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->tolerance);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_max_accelerations_scaling_factor;
      u_max_accelerations_scaling_factor.base = 0;
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_accelerations_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_accelerations_scaling_factor = u_max_accelerations_scaling_factor.real;
      offset += sizeof(this->max_accelerations_scaling_factor);
      union {
        double real;
        uint64_t base;
      } u_max_velocity_scaling_factor;
      u_max_velocity_scaling_factor.base = 0;
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_max_velocity_scaling_factor.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->max_velocity_scaling_factor = u_max_velocity_scaling_factor.real;
      offset += sizeof(this->max_velocity_scaling_factor);
      union {
        double real;
        uint64_t base;
      } u_tolerance;
      u_tolerance.base = 0;
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_tolerance.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->tolerance = u_tolerance.real;
      offset += sizeof(this->tolerance);
     return offset;
    }

    const char * getType(){ return "open_manipulator_msgs/KinematicsPose"; };
    const char * getMD5(){ return "bad8d5def2efabb0336490f8e9f6f2e2"; };

  };

}
#endif
