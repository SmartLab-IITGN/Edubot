#ifndef _ROS_SERVICE_SetStampedPoseParam_h
#define _ROS_SERVICE_SetStampedPoseParam_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseStamped.h"

namespace dead_reckoning
{

static const char SETSTAMPEDPOSEPARAM[] = "dead_reckoning/SetStampedPoseParam";

  class SetStampedPoseParamRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::PoseStamped _pose_type;
      _pose_type pose;

    SetStampedPoseParamRequest():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETSTAMPEDPOSEPARAM; };
    virtual const char * getMD5() override { return "3f8930d968a3e84d471dff917bb1cdae"; };

  };

  class SetStampedPoseParamResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    SetStampedPoseParamResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return SETSTAMPEDPOSEPARAM; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class SetStampedPoseParam {
    public:
    typedef SetStampedPoseParamRequest Request;
    typedef SetStampedPoseParamResponse Response;
  };

}
#endif
