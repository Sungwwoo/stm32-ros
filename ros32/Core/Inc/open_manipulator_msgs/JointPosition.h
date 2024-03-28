#ifndef _ROS_open_manipulator_msgs_JointPosition_h
#define _ROS_open_manipulator_msgs_JointPosition_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_manipulator_msgs
{

  class JointPosition : public ros::Msg
  {
    public:
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;
      uint32_t position_length;
      typedef double _position_type;
      _position_type st_position;
      _position_type * position;
      typedef double _max_accelerations_scaling_factor_type;
      _max_accelerations_scaling_factor_type max_accelerations_scaling_factor;
      typedef double _max_velocity_scaling_factor_type;
      _max_velocity_scaling_factor_type max_velocity_scaling_factor;

    JointPosition():
      joint_name_length(0), joint_name(NULL),
      position_length(0), position(NULL),
      max_accelerations_scaling_factor(0),
      max_velocity_scaling_factor(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->joint_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_name_length);
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_joint_namei = strlen(this->joint_name[i]);
      varToArr(outbuffer + offset, length_joint_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name[i], length_joint_namei);
      offset += length_joint_namei;
      }
      *(outbuffer + offset + 0) = (this->position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_length);
      for( uint32_t i = 0; i < position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_positioni;
      u_positioni.real = this->position[i];
      *(outbuffer + offset + 0) = (u_positioni.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_positioni.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_positioni.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_positioni.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_positioni.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_positioni.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_positioni.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_positioni.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->position[i]);
      }
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
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t joint_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_name_length);
      if(joint_name_lengthT > joint_name_length)
        this->joint_name = (char**)realloc(this->joint_name, joint_name_lengthT * sizeof(char*));
      joint_name_length = joint_name_lengthT;
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_st_joint_name;
      arrToVar(length_st_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_name-1]=0;
      this->st_joint_name = (char *)(inbuffer + offset-1);
      offset += length_st_joint_name;
        memcpy( &(this->joint_name[i]), &(this->st_joint_name), sizeof(char*));
      }
      uint32_t position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_length);
      if(position_lengthT > position_length)
        this->position = (double*)realloc(this->position, position_lengthT * sizeof(double));
      position_length = position_lengthT;
      for( uint32_t i = 0; i < position_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_position;
      u_st_position.base = 0;
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_position.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_position = u_st_position.real;
      offset += sizeof(this->st_position);
        memcpy( &(this->position[i]), &(this->st_position), sizeof(double));
      }
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
     return offset;
    }

    const char * getType(){ return "open_manipulator_msgs/JointPosition"; };
    const char * getMD5(){ return "b6b6bc3417b5da955b766eb41a6c1698"; };

  };

}
#endif
