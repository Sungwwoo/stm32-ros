#ifndef _ROS_mesh_msgs_Feature_h
#define _ROS_mesh_msgs_Feature_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float32.h"

namespace mesh_msgs
{

  class Feature : public ros::Msg
  {
    public:
      typedef geometry_msgs::Point _location_type;
      _location_type location;
      uint32_t descriptor_length;
      typedef std_msgs::Float32 _descriptor_type;
      _descriptor_type st_descriptor;
      _descriptor_type * descriptor;

    Feature():
      location(),
      descriptor_length(0), descriptor(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->location.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->descriptor_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->descriptor_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->descriptor_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->descriptor_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->descriptor_length);
      for( uint32_t i = 0; i < descriptor_length; i++){
      offset += this->descriptor[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->location.deserialize(inbuffer + offset);
      uint32_t descriptor_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      descriptor_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      descriptor_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      descriptor_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->descriptor_length);
      if(descriptor_lengthT > descriptor_length)
        this->descriptor = (std_msgs::Float32*)realloc(this->descriptor, descriptor_lengthT * sizeof(std_msgs::Float32));
      descriptor_length = descriptor_lengthT;
      for( uint32_t i = 0; i < descriptor_length; i++){
      offset += this->st_descriptor.deserialize(inbuffer + offset);
        memcpy( &(this->descriptor[i]), &(this->st_descriptor), sizeof(std_msgs::Float32));
      }
     return offset;
    }

    const char * getType(){ return "mesh_msgs/Feature"; };
    const char * getMD5(){ return "ac711cf3ef6eb8582240a7afe5b9a573"; };

  };

}
#endif
