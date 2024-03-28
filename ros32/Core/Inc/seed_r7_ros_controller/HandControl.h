#ifndef _ROS_SERVICE_HandControl_h
#define _ROS_SERVICE_HandControl_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace seed_r7_ros_controller
{

static const char HANDCONTROL[] = "seed_r7_ros_controller/HandControl";

  class HandControlRequest : public ros::Msg
  {
    public:
      typedef uint8_t _position_type;
      _position_type position;
      typedef const char* _script_type;
      _script_type script;
      typedef uint8_t _current_type;
      _current_type current;
      enum { POSITION_RIGHT =  0 };
      enum { POSITION_LEFT =  1 };
      enum { SCRIPT_GRASP =  grasp };
      enum { SCRIPT_RELEASE =  release };
      enum { SCRIPT_CANCEL =  cancel };

    HandControlRequest():
      position(0),
      script(""),
      current(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->position >> (8 * 0)) & 0xFF;
      offset += sizeof(this->position);
      uint32_t length_script = strlen(this->script);
      varToArr(outbuffer + offset, length_script);
      offset += 4;
      memcpy(outbuffer + offset, this->script, length_script);
      offset += length_script;
      *(outbuffer + offset + 0) = (this->current >> (8 * 0)) & 0xFF;
      offset += sizeof(this->current);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->position =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->position);
      uint32_t length_script;
      arrToVar(length_script, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_script; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_script-1]=0;
      this->script = (char *)(inbuffer + offset-1);
      offset += length_script;
      this->current =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->current);
     return offset;
    }

    const char * getType(){ return HANDCONTROL; };
    const char * getMD5(){ return "5b8557444e21ab5689c6d4cf096e0f7e"; };

  };

  class HandControlResponse : public ros::Msg
  {
    public:
      typedef const char* _result_type;
      _result_type result;

    HandControlResponse():
      result("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_result = strlen(this->result);
      varToArr(outbuffer + offset, length_result);
      offset += 4;
      memcpy(outbuffer + offset, this->result, length_result);
      offset += length_result;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_result;
      arrToVar(length_result, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_result; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_result-1]=0;
      this->result = (char *)(inbuffer + offset-1);
      offset += length_result;
     return offset;
    }

    const char * getType(){ return HANDCONTROL; };
    const char * getMD5(){ return "c22f2a1ed8654a0b365f1bb3f7ff2c0f"; };

  };

  class HandControl {
    public:
    typedef HandControlRequest Request;
    typedef HandControlResponse Response;
  };

}
#endif
