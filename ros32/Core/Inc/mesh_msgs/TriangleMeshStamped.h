#ifndef _ROS_mesh_msgs_TriangleMeshStamped_h
#define _ROS_mesh_msgs_TriangleMeshStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "mesh_msgs/TriangleMesh.h"

namespace mesh_msgs
{

  class TriangleMeshStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef mesh_msgs::TriangleMesh _mesh_type;
      _mesh_type mesh;

    TriangleMeshStamped():
      header(),
      mesh()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->mesh.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->mesh.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "mesh_msgs/TriangleMeshStamped"; };
    const char * getMD5(){ return "3e766dd12107291d682eb5e6c7442b9d"; };

  };

}
#endif
