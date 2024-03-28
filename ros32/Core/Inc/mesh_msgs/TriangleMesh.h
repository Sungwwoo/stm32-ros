#ifndef _ROS_mesh_msgs_TriangleMesh_h
#define _ROS_mesh_msgs_TriangleMesh_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mesh_msgs/TriangleIndices.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include "mesh_msgs/MeshMaterial.h"
#include "sensor_msgs/Image.h"
#include "mesh_msgs/MeshFaceCluster.h"

namespace mesh_msgs
{

  class TriangleMesh : public ros::Msg
  {
    public:
      uint32_t triangles_length;
      typedef mesh_msgs::TriangleIndices _triangles_type;
      _triangles_type st_triangles;
      _triangles_type * triangles;
      uint32_t vertices_length;
      typedef geometry_msgs::Point _vertices_type;
      _vertices_type st_vertices;
      _vertices_type * vertices;
      uint32_t vertex_normals_length;
      typedef geometry_msgs::Point _vertex_normals_type;
      _vertex_normals_type st_vertex_normals;
      _vertex_normals_type * vertex_normals;
      uint32_t vertex_colors_length;
      typedef std_msgs::ColorRGBA _vertex_colors_type;
      _vertex_colors_type st_vertex_colors;
      _vertex_colors_type * vertex_colors;
      uint32_t triangle_colors_length;
      typedef std_msgs::ColorRGBA _triangle_colors_type;
      _triangle_colors_type st_triangle_colors;
      _triangle_colors_type * triangle_colors;
      uint32_t vertex_texture_coords_length;
      typedef geometry_msgs::Point _vertex_texture_coords_type;
      _vertex_texture_coords_type st_vertex_texture_coords;
      _vertex_texture_coords_type * vertex_texture_coords;
      uint32_t face_materials_length;
      typedef mesh_msgs::MeshMaterial _face_materials_type;
      _face_materials_type st_face_materials;
      _face_materials_type * face_materials;
      uint32_t textures_length;
      typedef sensor_msgs::Image _textures_type;
      _textures_type st_textures;
      _textures_type * textures;
      uint32_t clusters_length;
      typedef mesh_msgs::MeshFaceCluster _clusters_type;
      _clusters_type st_clusters;
      _clusters_type * clusters;

    TriangleMesh():
      triangles_length(0), triangles(NULL),
      vertices_length(0), vertices(NULL),
      vertex_normals_length(0), vertex_normals(NULL),
      vertex_colors_length(0), vertex_colors(NULL),
      triangle_colors_length(0), triangle_colors(NULL),
      vertex_texture_coords_length(0), vertex_texture_coords(NULL),
      face_materials_length(0), face_materials(NULL),
      textures_length(0), textures(NULL),
      clusters_length(0), clusters(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->triangles_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->triangles_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->triangles_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->triangles_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->triangles_length);
      for( uint32_t i = 0; i < triangles_length; i++){
      offset += this->triangles[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->vertices_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertices_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertices_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertices_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertices_length);
      for( uint32_t i = 0; i < vertices_length; i++){
      offset += this->vertices[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->vertex_normals_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertex_normals_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertex_normals_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertex_normals_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertex_normals_length);
      for( uint32_t i = 0; i < vertex_normals_length; i++){
      offset += this->vertex_normals[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->vertex_colors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertex_colors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertex_colors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertex_colors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertex_colors_length);
      for( uint32_t i = 0; i < vertex_colors_length; i++){
      offset += this->vertex_colors[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->triangle_colors_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->triangle_colors_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->triangle_colors_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->triangle_colors_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->triangle_colors_length);
      for( uint32_t i = 0; i < triangle_colors_length; i++){
      offset += this->triangle_colors[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->vertex_texture_coords_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertex_texture_coords_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertex_texture_coords_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertex_texture_coords_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertex_texture_coords_length);
      for( uint32_t i = 0; i < vertex_texture_coords_length; i++){
      offset += this->vertex_texture_coords[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->face_materials_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->face_materials_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->face_materials_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->face_materials_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->face_materials_length);
      for( uint32_t i = 0; i < face_materials_length; i++){
      offset += this->face_materials[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->textures_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->textures_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->textures_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->textures_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->textures_length);
      for( uint32_t i = 0; i < textures_length; i++){
      offset += this->textures[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset + 0) = (this->clusters_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->clusters_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->clusters_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->clusters_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clusters_length);
      for( uint32_t i = 0; i < clusters_length; i++){
      offset += this->clusters[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t triangles_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      triangles_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      triangles_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      triangles_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->triangles_length);
      if(triangles_lengthT > triangles_length)
        this->triangles = (mesh_msgs::TriangleIndices*)realloc(this->triangles, triangles_lengthT * sizeof(mesh_msgs::TriangleIndices));
      triangles_length = triangles_lengthT;
      for( uint32_t i = 0; i < triangles_length; i++){
      offset += this->st_triangles.deserialize(inbuffer + offset);
        memcpy( &(this->triangles[i]), &(this->st_triangles), sizeof(mesh_msgs::TriangleIndices));
      }
      uint32_t vertices_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vertices_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vertices_length);
      if(vertices_lengthT > vertices_length)
        this->vertices = (geometry_msgs::Point*)realloc(this->vertices, vertices_lengthT * sizeof(geometry_msgs::Point));
      vertices_length = vertices_lengthT;
      for( uint32_t i = 0; i < vertices_length; i++){
      offset += this->st_vertices.deserialize(inbuffer + offset);
        memcpy( &(this->vertices[i]), &(this->st_vertices), sizeof(geometry_msgs::Point));
      }
      uint32_t vertex_normals_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vertex_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vertex_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vertex_normals_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vertex_normals_length);
      if(vertex_normals_lengthT > vertex_normals_length)
        this->vertex_normals = (geometry_msgs::Point*)realloc(this->vertex_normals, vertex_normals_lengthT * sizeof(geometry_msgs::Point));
      vertex_normals_length = vertex_normals_lengthT;
      for( uint32_t i = 0; i < vertex_normals_length; i++){
      offset += this->st_vertex_normals.deserialize(inbuffer + offset);
        memcpy( &(this->vertex_normals[i]), &(this->st_vertex_normals), sizeof(geometry_msgs::Point));
      }
      uint32_t vertex_colors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vertex_colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vertex_colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vertex_colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vertex_colors_length);
      if(vertex_colors_lengthT > vertex_colors_length)
        this->vertex_colors = (std_msgs::ColorRGBA*)realloc(this->vertex_colors, vertex_colors_lengthT * sizeof(std_msgs::ColorRGBA));
      vertex_colors_length = vertex_colors_lengthT;
      for( uint32_t i = 0; i < vertex_colors_length; i++){
      offset += this->st_vertex_colors.deserialize(inbuffer + offset);
        memcpy( &(this->vertex_colors[i]), &(this->st_vertex_colors), sizeof(std_msgs::ColorRGBA));
      }
      uint32_t triangle_colors_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      triangle_colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      triangle_colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      triangle_colors_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->triangle_colors_length);
      if(triangle_colors_lengthT > triangle_colors_length)
        this->triangle_colors = (std_msgs::ColorRGBA*)realloc(this->triangle_colors, triangle_colors_lengthT * sizeof(std_msgs::ColorRGBA));
      triangle_colors_length = triangle_colors_lengthT;
      for( uint32_t i = 0; i < triangle_colors_length; i++){
      offset += this->st_triangle_colors.deserialize(inbuffer + offset);
        memcpy( &(this->triangle_colors[i]), &(this->st_triangle_colors), sizeof(std_msgs::ColorRGBA));
      }
      uint32_t vertex_texture_coords_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      vertex_texture_coords_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      vertex_texture_coords_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      vertex_texture_coords_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->vertex_texture_coords_length);
      if(vertex_texture_coords_lengthT > vertex_texture_coords_length)
        this->vertex_texture_coords = (geometry_msgs::Point*)realloc(this->vertex_texture_coords, vertex_texture_coords_lengthT * sizeof(geometry_msgs::Point));
      vertex_texture_coords_length = vertex_texture_coords_lengthT;
      for( uint32_t i = 0; i < vertex_texture_coords_length; i++){
      offset += this->st_vertex_texture_coords.deserialize(inbuffer + offset);
        memcpy( &(this->vertex_texture_coords[i]), &(this->st_vertex_texture_coords), sizeof(geometry_msgs::Point));
      }
      uint32_t face_materials_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      face_materials_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      face_materials_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      face_materials_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->face_materials_length);
      if(face_materials_lengthT > face_materials_length)
        this->face_materials = (mesh_msgs::MeshMaterial*)realloc(this->face_materials, face_materials_lengthT * sizeof(mesh_msgs::MeshMaterial));
      face_materials_length = face_materials_lengthT;
      for( uint32_t i = 0; i < face_materials_length; i++){
      offset += this->st_face_materials.deserialize(inbuffer + offset);
        memcpy( &(this->face_materials[i]), &(this->st_face_materials), sizeof(mesh_msgs::MeshMaterial));
      }
      uint32_t textures_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      textures_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      textures_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      textures_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->textures_length);
      if(textures_lengthT > textures_length)
        this->textures = (sensor_msgs::Image*)realloc(this->textures, textures_lengthT * sizeof(sensor_msgs::Image));
      textures_length = textures_lengthT;
      for( uint32_t i = 0; i < textures_length; i++){
      offset += this->st_textures.deserialize(inbuffer + offset);
        memcpy( &(this->textures[i]), &(this->st_textures), sizeof(sensor_msgs::Image));
      }
      uint32_t clusters_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      clusters_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->clusters_length);
      if(clusters_lengthT > clusters_length)
        this->clusters = (mesh_msgs::MeshFaceCluster*)realloc(this->clusters, clusters_lengthT * sizeof(mesh_msgs::MeshFaceCluster));
      clusters_length = clusters_lengthT;
      for( uint32_t i = 0; i < clusters_length; i++){
      offset += this->st_clusters.deserialize(inbuffer + offset);
        memcpy( &(this->clusters[i]), &(this->st_clusters), sizeof(mesh_msgs::MeshFaceCluster));
      }
     return offset;
    }

    const char * getType(){ return "mesh_msgs/TriangleMesh"; };
    const char * getMD5(){ return "b112c5b670c2c3e8b1571aae11ccc3da"; };

  };

}
#endif
