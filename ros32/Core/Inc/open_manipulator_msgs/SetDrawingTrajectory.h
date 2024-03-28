#ifndef _ROS_SERVICE_SetDrawingTrajectory_h
#define _ROS_SERVICE_SetDrawingTrajectory_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace open_manipulator_msgs
{

static const char SETDRAWINGTRAJECTORY[] = "open_manipulator_msgs/SetDrawingTrajectory";

  class SetDrawingTrajectoryRequest : public ros::Msg
  {
    public:
      typedef const char* _end_effector_name_type;
      _end_effector_name_type end_effector_name;
      typedef const char* _drawing_trajectory_name_type;
      _drawing_trajectory_name_type drawing_trajectory_name;
      uint32_t param_length;
      typedef double _param_type;
      _param_type st_param;
      _param_type * param;
      typedef double _path_time_type;
      _path_time_type path_time;

    SetDrawingTrajectoryRequest():
      end_effector_name(""),
      drawing_trajectory_name(""),
      param_length(0), param(NULL),
      path_time(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_end_effector_name = strlen(this->end_effector_name);
      varToArr(outbuffer + offset, length_end_effector_name);
      offset += 4;
      memcpy(outbuffer + offset, this->end_effector_name, length_end_effector_name);
      offset += length_end_effector_name;
      uint32_t length_drawing_trajectory_name = strlen(this->drawing_trajectory_name);
      varToArr(outbuffer + offset, length_drawing_trajectory_name);
      offset += 4;
      memcpy(outbuffer + offset, this->drawing_trajectory_name, length_drawing_trajectory_name);
      offset += length_drawing_trajectory_name;
      *(outbuffer + offset + 0) = (this->param_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->param_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->param_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->param_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->param_length);
      for( uint32_t i = 0; i < param_length; i++){
      union {
        double real;
        uint64_t base;
      } u_parami;
      u_parami.real = this->param[i];
      *(outbuffer + offset + 0) = (u_parami.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_parami.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_parami.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_parami.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_parami.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_parami.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_parami.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_parami.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->param[i]);
      }
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
      uint32_t length_end_effector_name;
      arrToVar(length_end_effector_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_end_effector_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_end_effector_name-1]=0;
      this->end_effector_name = (char *)(inbuffer + offset-1);
      offset += length_end_effector_name;
      uint32_t length_drawing_trajectory_name;
      arrToVar(length_drawing_trajectory_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_drawing_trajectory_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_drawing_trajectory_name-1]=0;
      this->drawing_trajectory_name = (char *)(inbuffer + offset-1);
      offset += length_drawing_trajectory_name;
      uint32_t param_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      param_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      param_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      param_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->param_length);
      if(param_lengthT > param_length)
        this->param = (double*)realloc(this->param, param_lengthT * sizeof(double));
      param_length = param_lengthT;
      for( uint32_t i = 0; i < param_length; i++){
      union {
        double real;
        uint64_t base;
      } u_st_param;
      u_st_param.base = 0;
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_st_param.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->st_param = u_st_param.real;
      offset += sizeof(this->st_param);
        memcpy( &(this->param[i]), &(this->st_param), sizeof(double));
      }
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

    const char * getType(){ return SETDRAWINGTRAJECTORY; };
    const char * getMD5(){ return "5b1621cd6a6a57a64c9ee8bfb64e3d14"; };

  };

  class SetDrawingTrajectoryResponse : public ros::Msg
  {
    public:
      typedef bool _is_planned_type;
      _is_planned_type is_planned;

    SetDrawingTrajectoryResponse():
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

    const char * getType(){ return SETDRAWINGTRAJECTORY; };
    const char * getMD5(){ return "2638cc2443b1469b0e9e152083d7128d"; };

  };

  class SetDrawingTrajectory {
    public:
    typedef SetDrawingTrajectoryRequest Request;
    typedef SetDrawingTrajectoryResponse Response;
  };

}
#endif
