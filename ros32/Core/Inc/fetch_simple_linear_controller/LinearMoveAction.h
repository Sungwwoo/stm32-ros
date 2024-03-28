#ifndef _ROS_fetch_simple_linear_controller_LinearMoveAction_h
#define _ROS_fetch_simple_linear_controller_LinearMoveAction_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "fetch_simple_linear_controller/LinearMoveActionGoal.h"
#include "fetch_simple_linear_controller/LinearMoveActionResult.h"
#include "fetch_simple_linear_controller/LinearMoveActionFeedback.h"

namespace fetch_simple_linear_controller
{

  class LinearMoveAction : public ros::Msg
  {
    public:
      typedef fetch_simple_linear_controller::LinearMoveActionGoal _action_goal_type;
      _action_goal_type action_goal;
      typedef fetch_simple_linear_controller::LinearMoveActionResult _action_result_type;
      _action_result_type action_result;
      typedef fetch_simple_linear_controller::LinearMoveActionFeedback _action_feedback_type;
      _action_feedback_type action_feedback;

    LinearMoveAction():
      action_goal(),
      action_result(),
      action_feedback()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->action_goal.serialize(outbuffer + offset);
      offset += this->action_result.serialize(outbuffer + offset);
      offset += this->action_feedback.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->action_goal.deserialize(inbuffer + offset);
      offset += this->action_result.deserialize(inbuffer + offset);
      offset += this->action_feedback.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "fetch_simple_linear_controller/LinearMoveAction"; };
    const char * getMD5(){ return "23a60b77de92f470a359491a472a1760"; };

  };

}
#endif
