#ifndef _ROS_SERVICE_SetKinematicsPose_h
#define _ROS_SERVICE_SetKinematicsPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "open_manipulator_msgs/KinematicsPose.h"

namespace open_manipulator_msgs
{

static const char SETKINEMATICSPOSE[] = "open_manipulator_msgs/SetKinematicsPose";

  class SetKinematicsPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _planning_group_type;
      _planning_group_type planning_group;
      typedef const char* _end_effector_name_type;
      _end_effector_name_type end_effector_name;
      typedef open_manipulator_msgs::KinematicsPose _kinematics_pose_type;
      _kinematics_pose_type kinematics_pose;
      typedef double _path_time_type;
      _path_time_type path_time;

    SetKinematicsPoseRequest():
      planning_group(""),
      end_effector_name(""),
      kinematics_pose(),
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
      uint32_t length_end_effector_name = strlen(this->end_effector_name);
      varToArr(outbuffer + offset, length_end_effector_name);
      offset += 4;
      memcpy(outbuffer + offset, this->end_effector_name, length_end_effector_name);
      offset += length_end_effector_name;
      offset += this->kinematics_pose.serialize(outbuffer + offset);
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
      uint32_t length_end_effector_name;
      arrToVar(length_end_effector_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_end_effector_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_end_effector_name-1]=0;
      this->end_effector_name = (char *)(inbuffer + offset-1);
      offset += length_end_effector_name;
      offset += this->kinematics_pose.deserialize(inbuffer + offset);
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

    const char * getType(){ return SETKINEMATICSPOSE; };
    const char * getMD5(){ return "c4791502d3cd986f50c19faec2e660dc"; };

  };

  class SetKinematicsPoseResponse : public ros::Msg
  {
    public:
      typedef bool _is_planned_type;
      _is_planned_type is_planned;

    SetKinematicsPoseResponse():
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

    const char * getType(){ return SETKINEMATICSPOSE; };
    const char * getMD5(){ return "2638cc2443b1469b0e9e152083d7128d"; };

  };

  class SetKinematicsPose {
    public:
    typedef SetKinematicsPoseRequest Request;
    typedef SetKinematicsPoseResponse Response;
  };

}
#endif
