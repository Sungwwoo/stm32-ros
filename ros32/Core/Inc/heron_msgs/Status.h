#ifndef _ROS_heron_msgs_Status_h
#define _ROS_heron_msgs_Status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "ros/duration.h"

namespace heron_msgs
{

  class Status : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef const char* _hardware_id_type;
      _hardware_id_type hardware_id;
      typedef ros::Duration _mcu_uptime_type;
      _mcu_uptime_type mcu_uptime;
      typedef ros::Duration _connection_uptime_type;
      _connection_uptime_type connection_uptime;
      typedef float _pcb_temperature_type;
      _pcb_temperature_type pcb_temperature;
      typedef float _user_current_type;
      _user_current_type user_current;
      typedef float _user_power_consumed_type;
      _user_power_consumed_type user_power_consumed;
      typedef float _motor_power_consumed_type;
      _motor_power_consumed_type motor_power_consumed;
      typedef float _total_power_consumed_type;
      _total_power_consumed_type total_power_consumed;

    Status():
      header(),
      hardware_id(""),
      mcu_uptime(),
      connection_uptime(),
      pcb_temperature(0),
      user_current(0),
      user_power_consumed(0),
      motor_power_consumed(0),
      total_power_consumed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      uint32_t length_hardware_id = strlen(this->hardware_id);
      varToArr(outbuffer + offset, length_hardware_id);
      offset += 4;
      memcpy(outbuffer + offset, this->hardware_id, length_hardware_id);
      offset += length_hardware_id;
      *(outbuffer + offset + 0) = (this->mcu_uptime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mcu_uptime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mcu_uptime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mcu_uptime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mcu_uptime.sec);
      *(outbuffer + offset + 0) = (this->mcu_uptime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->mcu_uptime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->mcu_uptime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->mcu_uptime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->mcu_uptime.nsec);
      *(outbuffer + offset + 0) = (this->connection_uptime.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_uptime.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_uptime.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_uptime.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_uptime.sec);
      *(outbuffer + offset + 0) = (this->connection_uptime.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->connection_uptime.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->connection_uptime.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->connection_uptime.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->connection_uptime.nsec);
      union {
        float real;
        uint32_t base;
      } u_pcb_temperature;
      u_pcb_temperature.real = this->pcb_temperature;
      *(outbuffer + offset + 0) = (u_pcb_temperature.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pcb_temperature.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pcb_temperature.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pcb_temperature.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pcb_temperature);
      union {
        float real;
        uint32_t base;
      } u_user_current;
      u_user_current.real = this->user_current;
      *(outbuffer + offset + 0) = (u_user_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_user_current.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_user_current.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_user_current.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->user_current);
      union {
        float real;
        uint32_t base;
      } u_user_power_consumed;
      u_user_power_consumed.real = this->user_power_consumed;
      *(outbuffer + offset + 0) = (u_user_power_consumed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_user_power_consumed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_user_power_consumed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_user_power_consumed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->user_power_consumed);
      union {
        float real;
        uint32_t base;
      } u_motor_power_consumed;
      u_motor_power_consumed.real = this->motor_power_consumed;
      *(outbuffer + offset + 0) = (u_motor_power_consumed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_motor_power_consumed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_motor_power_consumed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_motor_power_consumed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_power_consumed);
      union {
        float real;
        uint32_t base;
      } u_total_power_consumed;
      u_total_power_consumed.real = this->total_power_consumed;
      *(outbuffer + offset + 0) = (u_total_power_consumed.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_total_power_consumed.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_total_power_consumed.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_total_power_consumed.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->total_power_consumed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t length_hardware_id;
      arrToVar(length_hardware_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_hardware_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_hardware_id-1]=0;
      this->hardware_id = (char *)(inbuffer + offset-1);
      offset += length_hardware_id;
      this->mcu_uptime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mcu_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mcu_uptime.sec);
      this->mcu_uptime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->mcu_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->mcu_uptime.nsec);
      this->connection_uptime.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_uptime.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_uptime.sec);
      this->connection_uptime.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->connection_uptime.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->connection_uptime.nsec);
      union {
        float real;
        uint32_t base;
      } u_pcb_temperature;
      u_pcb_temperature.base = 0;
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pcb_temperature.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pcb_temperature = u_pcb_temperature.real;
      offset += sizeof(this->pcb_temperature);
      union {
        float real;
        uint32_t base;
      } u_user_current;
      u_user_current.base = 0;
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_user_current.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->user_current = u_user_current.real;
      offset += sizeof(this->user_current);
      union {
        float real;
        uint32_t base;
      } u_user_power_consumed;
      u_user_power_consumed.base = 0;
      u_user_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_user_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_user_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_user_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->user_power_consumed = u_user_power_consumed.real;
      offset += sizeof(this->user_power_consumed);
      union {
        float real;
        uint32_t base;
      } u_motor_power_consumed;
      u_motor_power_consumed.base = 0;
      u_motor_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_motor_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_motor_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_motor_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->motor_power_consumed = u_motor_power_consumed.real;
      offset += sizeof(this->motor_power_consumed);
      union {
        float real;
        uint32_t base;
      } u_total_power_consumed;
      u_total_power_consumed.base = 0;
      u_total_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_total_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_total_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_total_power_consumed.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->total_power_consumed = u_total_power_consumed.real;
      offset += sizeof(this->total_power_consumed);
     return offset;
    }

    const char * getType(){ return "heron_msgs/Status"; };
    const char * getMD5(){ return "73638ba99aee6dc46e9610079bfb59d8"; };

  };

}
#endif
