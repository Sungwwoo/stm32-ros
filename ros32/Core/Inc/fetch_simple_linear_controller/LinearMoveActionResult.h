#ifndef _ROS_fetch_simple_linear_controller_LinearMoveActionResult_h
#define _ROS_fetch_simple_linear_controller_LinearMoveActionResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "actionlib_msgs/GoalStatus.h"
#include "fetch_simple_linear_controller/LinearMoveResult.h"

namespace fetch_simple_linear_controller
{

  class LinearMoveActionResult : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef actionlib_msgs::GoalStatus _status_type;
      _status_type status;
      typedef fetch_simple_linear_controller::LinearMoveResult _result_type;
      _result_type result;

    LinearMoveActionResult():
      header(),
      status(),
      result()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->status.serialize(outbuffer + offset);
      offset += this->result.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->status.deserialize(inbuffer + offset);
      offset += this->result.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fetch_simple_linear_controller/LinearMoveActionResult"; };
    const char * getMD5(){ return "6c0eb433f5e1ab6ec91519097b9c1b9f"; };

  };

}
#endif
