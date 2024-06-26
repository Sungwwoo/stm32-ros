#ifndef _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoData_h
#define _ROS_pr2_gripper_sensor_msgs_PR2GripperForceServoData_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"
#include "pr2_gripper_sensor_msgs/PR2GripperSensorRTState.h"

namespace pr2_gripper_sensor_msgs
{

  class PR2GripperForceServoData : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef double _left_fingertip_pad_force_type;
      _left_fingertip_pad_force_type left_fingertip_pad_force;
      typedef double _right_fingertip_pad_force_type;
      _right_fingertip_pad_force_type right_fingertip_pad_force;
      typedef double _joint_effort_type;
      _joint_effort_type joint_effort;
      typedef bool _force_achieved_type;
      _force_achieved_type force_achieved;
      typedef pr2_gripper_sensor_msgs::PR2GripperSensorRTState _rtstate_type;
      _rtstate_type rtstate;

    PR2GripperForceServoData():
      stamp(),
      left_fingertip_pad_force(0),
      right_fingertip_pad_force(0),
      joint_effort(0),
      force_achieved(0),
      rtstate()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        double real;
        uint64_t base;
      } u_left_fingertip_pad_force;
      u_left_fingertip_pad_force.real = this->left_fingertip_pad_force;
      *(outbuffer + offset + 0) = (u_left_fingertip_pad_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left_fingertip_pad_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left_fingertip_pad_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left_fingertip_pad_force.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_left_fingertip_pad_force.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_left_fingertip_pad_force.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_left_fingertip_pad_force.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_left_fingertip_pad_force.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->left_fingertip_pad_force);
      union {
        double real;
        uint64_t base;
      } u_right_fingertip_pad_force;
      u_right_fingertip_pad_force.real = this->right_fingertip_pad_force;
      *(outbuffer + offset + 0) = (u_right_fingertip_pad_force.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right_fingertip_pad_force.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right_fingertip_pad_force.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right_fingertip_pad_force.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_right_fingertip_pad_force.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_right_fingertip_pad_force.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_right_fingertip_pad_force.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_right_fingertip_pad_force.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->right_fingertip_pad_force);
      union {
        double real;
        uint64_t base;
      } u_joint_effort;
      u_joint_effort.real = this->joint_effort;
      *(outbuffer + offset + 0) = (u_joint_effort.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_joint_effort.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_joint_effort.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_joint_effort.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_joint_effort.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_joint_effort.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_joint_effort.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_joint_effort.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->joint_effort);
      union {
        bool real;
        uint8_t base;
      } u_force_achieved;
      u_force_achieved.real = this->force_achieved;
      *(outbuffer + offset + 0) = (u_force_achieved.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->force_achieved);
      offset += this->rtstate.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        double real;
        uint64_t base;
      } u_left_fingertip_pad_force;
      u_left_fingertip_pad_force.base = 0;
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_left_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->left_fingertip_pad_force = u_left_fingertip_pad_force.real;
      offset += sizeof(this->left_fingertip_pad_force);
      union {
        double real;
        uint64_t base;
      } u_right_fingertip_pad_force;
      u_right_fingertip_pad_force.base = 0;
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_right_fingertip_pad_force.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->right_fingertip_pad_force = u_right_fingertip_pad_force.real;
      offset += sizeof(this->right_fingertip_pad_force);
      union {
        double real;
        uint64_t base;
      } u_joint_effort;
      u_joint_effort.base = 0;
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_joint_effort.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->joint_effort = u_joint_effort.real;
      offset += sizeof(this->joint_effort);
      union {
        bool real;
        uint8_t base;
      } u_force_achieved;
      u_force_achieved.base = 0;
      u_force_achieved.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->force_achieved = u_force_achieved.real;
      offset += sizeof(this->force_achieved);
      offset += this->rtstate.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pr2_gripper_sensor_msgs/PR2GripperForceServoData"; };
    const char * getMD5(){ return "d3960eb2ecb6a9b4c27065619e47fd06"; };

  };

}
#endif
