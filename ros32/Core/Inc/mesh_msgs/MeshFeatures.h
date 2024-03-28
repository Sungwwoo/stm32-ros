#ifndef _ROS_mesh_msgs_MeshFeatures_h
#define _ROS_mesh_msgs_MeshFeatures_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mesh_msgs/Feature.h"

namespace mesh_msgs
{

  class MeshFeatures : public ros::Msg
  {
    public:
      typedef const char* _map_uuid_type;
      _map_uuid_type map_uuid;
      uint32_t features_length;
      typedef mesh_msgs::Feature _features_type;
      _features_type st_features;
      _features_type * features;

    MeshFeatures():
      map_uuid(""),
      features_length(0), features(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_map_uuid = strlen(this->map_uuid);
      varToArr(outbuffer + offset, length_map_uuid);
      offset += 4;
      memcpy(outbuffer + offset, this->map_uuid, length_map_uuid);
      offset += length_map_uuid;
      *(outbuffer + offset + 0) = (this->features_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->features_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->features_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->features_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->features_length);
      for( uint32_t i = 0; i < features_length; i++){
      offset += this->features[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_map_uuid;
      arrToVar(length_map_uuid, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_map_uuid; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_map_uuid-1]=0;
      this->map_uuid = (char *)(inbuffer + offset-1);
      offset += length_map_uuid;
      uint32_t features_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      features_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->features_length);
      if(features_lengthT > features_length)
        this->features = (mesh_msgs::Feature*)realloc(this->features, features_lengthT * sizeof(mesh_msgs::Feature));
      features_length = features_lengthT;
      for( uint32_t i = 0; i < features_length; i++){
      offset += this->st_features.deserialize(inbuffer + offset);
        memcpy( &(this->features[i]), &(this->st_features), sizeof(mesh_msgs::Feature));
      }
     return offset;
    }

    const char * getType(){ return "mesh_msgs/MeshFeatures"; };
    const char * getMD5(){ return "ea0bfd1049bc24f2cd76d68461f1f987"; };

  };

}
#endif
