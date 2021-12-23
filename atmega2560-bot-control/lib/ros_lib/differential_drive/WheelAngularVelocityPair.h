#ifndef _ROS_differential_drive_WheelAngularVelocityPair_h
#define _ROS_differential_drive_WheelAngularVelocityPair_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace differential_drive
{

  class WheelAngularVelocityPair : public ros::Msg
  {
    public:
      typedef float _wheel_angular_velocity_left_type;
      _wheel_angular_velocity_left_type wheel_angular_velocity_left;
      typedef float _wheel_angular_velocity_right_type;
      _wheel_angular_velocity_right_type wheel_angular_velocity_right;

    WheelAngularVelocityPair():
      wheel_angular_velocity_left(0),
      wheel_angular_velocity_right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_angular_velocity_left;
      u_wheel_angular_velocity_left.real = this->wheel_angular_velocity_left;
      *(outbuffer + offset + 0) = (u_wheel_angular_velocity_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_angular_velocity_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_angular_velocity_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_angular_velocity_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_angular_velocity_left);
      union {
        float real;
        uint32_t base;
      } u_wheel_angular_velocity_right;
      u_wheel_angular_velocity_right.real = this->wheel_angular_velocity_right;
      *(outbuffer + offset + 0) = (u_wheel_angular_velocity_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_wheel_angular_velocity_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_wheel_angular_velocity_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_wheel_angular_velocity_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->wheel_angular_velocity_right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_wheel_angular_velocity_left;
      u_wheel_angular_velocity_left.base = 0;
      u_wheel_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_angular_velocity_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_angular_velocity_left = u_wheel_angular_velocity_left.real;
      offset += sizeof(this->wheel_angular_velocity_left);
      union {
        float real;
        uint32_t base;
      } u_wheel_angular_velocity_right;
      u_wheel_angular_velocity_right.base = 0;
      u_wheel_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_wheel_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_wheel_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_wheel_angular_velocity_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->wheel_angular_velocity_right = u_wheel_angular_velocity_right.real;
      offset += sizeof(this->wheel_angular_velocity_right);
     return offset;
    }

    virtual const char * getType() override { return "differential_drive/WheelAngularVelocityPair"; };
    virtual const char * getMD5() override { return "adff221a07855e72470c2f5460fcf2d6"; };

  };

}
#endif
