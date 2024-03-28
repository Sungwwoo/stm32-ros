#ifndef _ROS_md49_messages_md49_encoders_h
#define _ROS_md49_messages_md49_encoders_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace md49_messages
{

  class md49_encoders : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef int32_t _encoder_l_type;
      _encoder_l_type encoder_l;
      typedef int32_t _encoder_r_type;
      _encoder_r_type encoder_r;
      typedef int16_t _encoderbyte1l_type;
      _encoderbyte1l_type encoderbyte1l;
      typedef int16_t _encoderbyte2l_type;
      _encoderbyte2l_type encoderbyte2l;
      typedef int16_t _encoderbyte3l_type;
      _encoderbyte3l_type encoderbyte3l;
      typedef int16_t _encoderbyte4l_type;
      _encoderbyte4l_type encoderbyte4l;
      typedef int16_t _encoderbyte1r_type;
      _encoderbyte1r_type encoderbyte1r;
      typedef int16_t _encoderbyte2r_type;
      _encoderbyte2r_type encoderbyte2r;
      typedef int16_t _encoderbyte3r_type;
      _encoderbyte3r_type encoderbyte3r;
      typedef int16_t _encoderbyte4r_type;
      _encoderbyte4r_type encoderbyte4r;

    md49_encoders():
      header(),
      encoder_l(0),
      encoder_r(0),
      encoderbyte1l(0),
      encoderbyte2l(0),
      encoderbyte3l(0),
      encoderbyte4l(0),
      encoderbyte1r(0),
      encoderbyte2r(0),
      encoderbyte3r(0),
      encoderbyte4r(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_l;
      u_encoder_l.real = this->encoder_l;
      *(outbuffer + offset + 0) = (u_encoder_l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_l.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoder_l.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoder_l.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder_l);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_r;
      u_encoder_r.real = this->encoder_r;
      *(outbuffer + offset + 0) = (u_encoder_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoder_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_encoder_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_encoder_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->encoder_r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte1l;
      u_encoderbyte1l.real = this->encoderbyte1l;
      *(outbuffer + offset + 0) = (u_encoderbyte1l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte1l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte1l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte2l;
      u_encoderbyte2l.real = this->encoderbyte2l;
      *(outbuffer + offset + 0) = (u_encoderbyte2l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte2l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte2l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte3l;
      u_encoderbyte3l.real = this->encoderbyte3l;
      *(outbuffer + offset + 0) = (u_encoderbyte3l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte3l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte3l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte4l;
      u_encoderbyte4l.real = this->encoderbyte4l;
      *(outbuffer + offset + 0) = (u_encoderbyte4l.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte4l.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte4l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte1r;
      u_encoderbyte1r.real = this->encoderbyte1r;
      *(outbuffer + offset + 0) = (u_encoderbyte1r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte1r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte1r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte2r;
      u_encoderbyte2r.real = this->encoderbyte2r;
      *(outbuffer + offset + 0) = (u_encoderbyte2r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte2r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte2r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte3r;
      u_encoderbyte3r.real = this->encoderbyte3r;
      *(outbuffer + offset + 0) = (u_encoderbyte3r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte3r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte3r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte4r;
      u_encoderbyte4r.real = this->encoderbyte4r;
      *(outbuffer + offset + 0) = (u_encoderbyte4r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_encoderbyte4r.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->encoderbyte4r);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_l;
      u_encoder_l.base = 0;
      u_encoder_l.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_l.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoder_l.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoder_l.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoder_l = u_encoder_l.real;
      offset += sizeof(this->encoder_l);
      union {
        int32_t real;
        uint32_t base;
      } u_encoder_r;
      u_encoder_r.base = 0;
      u_encoder_r.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoder_r.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_encoder_r.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_encoder_r.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->encoder_r = u_encoder_r.real;
      offset += sizeof(this->encoder_r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte1l;
      u_encoderbyte1l.base = 0;
      u_encoderbyte1l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte1l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte1l = u_encoderbyte1l.real;
      offset += sizeof(this->encoderbyte1l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte2l;
      u_encoderbyte2l.base = 0;
      u_encoderbyte2l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte2l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte2l = u_encoderbyte2l.real;
      offset += sizeof(this->encoderbyte2l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte3l;
      u_encoderbyte3l.base = 0;
      u_encoderbyte3l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte3l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte3l = u_encoderbyte3l.real;
      offset += sizeof(this->encoderbyte3l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte4l;
      u_encoderbyte4l.base = 0;
      u_encoderbyte4l.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte4l.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte4l = u_encoderbyte4l.real;
      offset += sizeof(this->encoderbyte4l);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte1r;
      u_encoderbyte1r.base = 0;
      u_encoderbyte1r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte1r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte1r = u_encoderbyte1r.real;
      offset += sizeof(this->encoderbyte1r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte2r;
      u_encoderbyte2r.base = 0;
      u_encoderbyte2r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte2r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte2r = u_encoderbyte2r.real;
      offset += sizeof(this->encoderbyte2r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte3r;
      u_encoderbyte3r.base = 0;
      u_encoderbyte3r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte3r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte3r = u_encoderbyte3r.real;
      offset += sizeof(this->encoderbyte3r);
      union {
        int16_t real;
        uint16_t base;
      } u_encoderbyte4r;
      u_encoderbyte4r.base = 0;
      u_encoderbyte4r.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_encoderbyte4r.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->encoderbyte4r = u_encoderbyte4r.real;
      offset += sizeof(this->encoderbyte4r);
     return offset;
    }

    const char * getType(){ return "md49_messages/md49_encoders"; };
    const char * getMD5(){ return "36f14926d8d84c6fbc5fef36e1e33352"; };

  };

}
#endif
