#ifndef _ROS_md49_messages_md49_data_h
#define _ROS_md49_messages_md49_data_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace md49_messages
{

  class md49_data : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int16_t _speed_l_type;
      _speed_l_type speed_l;
      typedef int16_t _speed_r_type;
      _speed_r_type speed_r;
      typedef int16_t _volts_type;
      _volts_type volts;
      typedef int16_t _current_l_type;
      _current_l_type current_l;
      typedef int16_t _current_r_type;
      _current_r_type current_r;
      typedef int16_t _error_type;
      _error_type error;
      typedef int16_t _acceleration_type;
      _acceleration_type acceleration;
      typedef int16_t _mode_type;
      _mode_type mode;
      typedef int16_t _regulator_type;
      _regulator_type regulator;
      typedef int16_t _timeout_type;
      _timeout_type timeout;

    md49_data():
      header(),
      speed_l(0),
      speed_r(0),
      volts(0),
      current_l(0),
      current_r(0),
      error(0),
      acceleration(0),
      mode(0),
      regulator(0),
      timeout(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_speed_l;
      u_speed_l.real = this->speed_l;
      *(outbuffer + offset + 0) = (u_speed_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed_l);
      union {
        int16_t real;
        uint16_t base;
      } u_speed_r;
      u_speed_r.real = this->speed_r;
      *(outbuffer + offset + 0) = (u_speed_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->speed_r);
      union {
        int16_t real;
        uint16_t base;
      } u_volts;
      u_volts.real = this->volts;
      *(outbuffer + offset + 0) = (u_volts.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_volts.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->volts);
      union {
        int16_t real;
        uint16_t base;
      } u_current_l;
      u_current_l.real = this->current_l;
      *(outbuffer + offset + 0) = (u_current_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_l);
      union {
        int16_t real;
        uint16_t base;
      } u_current_r;
      u_current_r.real = this->current_r;
      *(outbuffer + offset + 0) = (u_current_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_current_r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->current_r);
      union {
        int16_t real;
        uint16_t base;
      } u_error;
      u_error.real = this->error;
      *(outbuffer + offset + 0) = (u_error.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_error.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->error);
      union {
        int16_t real;
        uint16_t base;
      } u_acceleration;
      u_acceleration.real = this->acceleration;
      *(outbuffer + offset + 0) = (u_acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_acceleration.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->acceleration);
      union {
        int16_t real;
        uint16_t base;
      } u_mode;
      u_mode.real = this->mode;
      *(outbuffer + offset + 0) = (u_mode.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_mode.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->mode);
      union {
        int16_t real;
        uint16_t base;
      } u_regulator;
      u_regulator.real = this->regulator;
      *(outbuffer + offset + 0) = (u_regulator.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_regulator.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->regulator);
      union {
        int16_t real;
        uint16_t base;
      } u_timeout;
      u_timeout.real = this->timeout;
      *(outbuffer + offset + 0) = (u_timeout.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_timeout.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->timeout);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int16_t real;
        uint16_t base;
      } u_speed_l;
      u_speed_l.base = 0;
      u_speed_l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_l = u_speed_l.real;
      offset += sizeof(this->speed_l);
      union {
        int16_t real;
        uint16_t base;
      } u_speed_r;
      u_speed_r.base = 0;
      u_speed_r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->speed_r = u_speed_r.real;
      offset += sizeof(this->speed_r);
      union {
        int16_t real;
        uint16_t base;
      } u_volts;
      u_volts.base = 0;
      u_volts.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_volts.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->volts = u_volts.real;
      offset += sizeof(this->volts);
      union {
        int16_t real;
        uint16_t base;
      } u_current_l;
      u_current_l.base = 0;
      u_current_l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_l = u_current_l.real;
      offset += sizeof(this->current_l);
      union {
        int16_t real;
        uint16_t base;
      } u_current_r;
      u_current_r.base = 0;
      u_current_r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_current_r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->current_r = u_current_r.real;
      offset += sizeof(this->current_r);
      union {
        int16_t real;
        uint16_t base;
      } u_error;
      u_error.base = 0;
      u_error.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_error.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->error = u_error.real;
      offset += sizeof(this->error);
      union {
        int16_t real;
        uint16_t base;
      } u_acceleration;
      u_acceleration.base = 0;
      u_acceleration.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_acceleration.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->acceleration = u_acceleration.real;
      offset += sizeof(this->acceleration);
      union {
        int16_t real;
        uint16_t base;
      } u_mode;
      u_mode.base = 0;
      u_mode.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_mode.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mode = u_mode.real;
      offset += sizeof(this->mode);
      union {
        int16_t real;
        uint16_t base;
      } u_regulator;
      u_regulator.base = 0;
      u_regulator.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_regulator.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->regulator = u_regulator.real;
      offset += sizeof(this->regulator);
      union {
        int16_t real;
        uint16_t base;
      } u_timeout;
      u_timeout.base = 0;
      u_timeout.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_timeout.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout = u_timeout.real;
      offset += sizeof(this->timeout);
     return offset;
    }

    const char * getType(){ return "md49_messages/md49_data"; };
    const char * getMD5(){ return "079df6b5900b9878c8ced1274ecb0bdf"; };

  };

}
#endif
