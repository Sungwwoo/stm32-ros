#ifndef _ROS_SERVICE_SetJointImpedance_h
#define _ROS_SERVICE_SetJointImpedance_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace franka_msgs
{

static const char SETJOINTIMPEDANCE[] = "franka_msgs/SetJointImpedance";

  class SetJointImpedanceRequest : public ros::Msg
  {
    public:
      double joint_stiffness[7];

    SetJointImpedanceRequest():
      joint_stiffness()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      for( uint32_t i = 0; i < 7; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_stiffnessi;
      u_joint_stiffnessi.real = this->joint_stiffness[i];
      *(outbuffer + offset + 0) = (u_joint_stiffnessi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_stiffnessi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_stiffnessi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_stiffnessi.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_stiffnessi.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_stiffnessi.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_stiffnessi.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_stiffnessi.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_stiffness[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      for( uint32_t i = 0; i < 7; i++){
      union {
        double real;
        uint64_t base;
      } u_joint_stiffnessi;
      u_joint_stiffnessi.base = 0;
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_joint_stiffnessi.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->joint_stiffness[i] = u_joint_stiffnessi.real;
      offset += sizeof(this->joint_stiffness[i]);
      }
     return offset;
    }

    const char * getType(){ return SETJOINTIMPEDANCE; };
    const char * getMD5(){ return "45e5482efb638f20554d876158c68e96"; };

  };

  class SetJointImpedanceResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _error_type;
      _error_type error;

    SetJointImpedanceResponse():
      success(0),
      error("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_error = strlen(this->error);
      varToArr(outbuffer + offset, length_error);
      offset += 4;
      memcpy(outbuffer + offset, this->error, length_error);
      offset += length_error;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_error;
      arrToVar(length_error, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_error; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_error-1]=0;
      this->error = (char *)(inbuffer + offset-1);
      offset += length_error;
     return offset;
    }

    const char * getType(){ return SETJOINTIMPEDANCE; };
    const char * getMD5(){ return "45872d25d65c97743cc71afc6d4e884d"; };

  };

  class SetJointImpedance {
    public:
    typedef SetJointImpedanceRequest Request;
    typedef SetJointImpedanceResponse Response;
  };

}
#endif
