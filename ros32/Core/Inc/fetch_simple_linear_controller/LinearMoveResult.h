#ifndef _ROS_fetch_simple_linear_controller_LinearMoveResult_h
#define _ROS_fetch_simple_linear_controller_LinearMoveResult_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Vector3.h"

namespace fetch_simple_linear_controller
{

  class LinearMoveResult : public ros::Msg
  {
    public:
      typedef geometry_msgs::Vector3 _error_type;
      _error_type error;

    LinearMoveResult():
      error()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->error.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->error.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fetch_simple_linear_controller/LinearMoveResult"; };
    const char * getMD5(){ return "ddcdcf876fa71cb72996e49795c08c06"; };

  };

}
#endif
