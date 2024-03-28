#ifndef _ROS_heron_msgs_Sense_h
#define _ROS_heron_msgs_Sense_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace heron_msgs
{

  class Sense : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef float _battery_type;
      _battery_type battery;
      typedef float _current_left_type;
      _current_left_type current_left;
      typedef float _current_right_type;
      _current_right_type current_right;
      typedef uint8_t _rc_type;
      _rc_type rc;
      typedef uint16_t _rc_throttle_type;
      _rc_throttle_type rc_throttle;
      typedef uint16_t _rc_rotation_type;
      _rc_rotation_type rc_rotation;
      typedef uint16_t _rc_enable_type;
      _rc_enable_type rc_enable;
      enum { RC_INRANGE = 1 };
      enum { RC_INUSE = 2 };

    Sense():
      header(),
      battery(0),
      current_left(0),
      current_right(0),
      rc(0),
      rc_throttle(0),
      rc_rotation(0),
      rc_enable(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.real = this->battery;
      *(outbuffer + offset + 0) = (u_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->battery);
      union {
        float real;
        uint32_t base;
      } u_current_left;
      u_current_left.real = this->current_left;
      *(outbuffer + offset + 0) = (u_current_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_left);
      union {
        float real;
        uint32_t base;
      } u_current_right;
      u_current_right.real = this->current_right;
      *(outbuffer + offset + 0) = (u_current_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_current_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_current_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->current_right);
      *(outbuffer + offset + 0) = (this->rc >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rc);
      *(outbuffer + offset + 0) = (this->rc_throttle >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_throttle >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_throttle);
      *(outbuffer + offset + 0) = (this->rc_rotation >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_rotation >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_rotation);
      *(outbuffer + offset + 0) = (this->rc_enable >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->rc_enable >> (8 * 1)) & 0xFF;
      offset += sizeof(this->rc_enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        uint32_t base;
      } u_battery;
      u_battery.base = 0;
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->battery = u_battery.real;
      offset += sizeof(this->battery);
      union {
        float real;
        uint32_t base;
      } u_current_left;
      u_current_left.base = 0;
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_left = u_current_left.real;
      offset += sizeof(this->current_left);
      union {
        float real;
        uint32_t base;
      } u_current_right;
      u_current_right.base = 0;
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_current_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->current_right = u_current_right.real;
      offset += sizeof(this->current_right);
      this->rc =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->rc);
      this->rc_throttle =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_throttle |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_throttle);
      this->rc_rotation =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_rotation |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_rotation);
      this->rc_enable =  ((uint16_t) (*(inbuffer + offset)));
      this->rc_enable |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->rc_enable);
     return offset;
    }

    const char * getType(){ return "heron_msgs/Sense"; };
    const char * getMD5(){ return "56d1cbddc5154c7883e86d1d6d7fe153"; };

  };

}
#endif
