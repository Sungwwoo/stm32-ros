#ifndef _ROS_SERVICE_SetJointPosition_h
#define _ROS_SERVICE_SetJointPosition_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "open_manipulator_msgs/JointPosition.h"

namespace open_manipulator_msgs
{

static const char SETJOINTPOSITION[] = "open_manipulator_msgs/SetJointPosition";

  class SetJointPositionRequest : public ros::Msg
  {
    public:
      typedef const char* _planning_group_type;
      _planning_group_type planning_group;
      typedef open_manipulator_msgs::JointPosition _joint_position_type;
      _joint_position_type joint_position;
      typedef double _path_time_type;
      _path_time_type path_time;

    SetJointPositionRequest():
      planning_group(""),
      joint_position(),
      path_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_planning_group = strlen(this->planning_group);
      varToArr(outbuffer + offset, length_planning_group);
      offset += 4;
      memcpy(outbuffer + offset, this->planning_group, length_planning_group);
      offset += length_planning_group;
      offset += this->joint_position.serialize(outbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_path_time;
      u_path_time.real = this->path_time;
      *(outbuffer + offset + 0) = (u_path_time.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_path_time.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_path_time.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_path_time.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_path_time.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_path_time.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_path_time.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_path_time.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->path_time);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_planning_group;
      arrToVar(length_planning_group, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_planning_group; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_planning_group-1]=0;
      this->planning_group = (char *)(inbuffer + offset-1);
      offset += length_planning_group;
      offset += this->joint_position.deserialize(inbuffer + offset);
      union {
        double real;
        uint64_t base;
      } u_path_time;
      u_path_time.base = 0;
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_path_time.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->path_time = u_path_time.real;
      offset += sizeof(this->path_time);
     return offset;
    }

    const char * getType(){ return SETJOINTPOSITION; };
    const char * getMD5(){ return "ab867938df63c0b7946cf0ff4eeddfcc"; };

  };

  class SetJointPositionResponse : public ros::Msg
  {
    public:
      typedef bool _is_planned_type;
      _is_planned_type is_planned;

    SetJointPositionResponse():
      is_planned(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_planned;
      u_is_planned.real = this->is_planned;
      *(outbuffer + offset + 0) = (u_is_planned.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_planned);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_is_planned;
      u_is_planned.base = 0;
      u_is_planned.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_planned = u_is_planned.real;
      offset += sizeof(this->is_planned);
     return offset;
    }

    const char * getType(){ return SETJOINTPOSITION; };
    const char * getMD5(){ return "2638cc2443b1469b0e9e152083d7128d"; };

  };

  class SetJointPosition {
    public:
    typedef SetJointPositionRequest Request;
    typedef SetJointPositionResponse Response;
  };

}
#endif
