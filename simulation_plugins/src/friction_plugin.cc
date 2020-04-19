#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include "gripper_controls/GetFriction.h"
#include "gripper_controls/Friction.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <thread>

namespace gazebo
{
  class ModelPush : public ModelPlugin{
  public:

    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/){
      this->model =_parent;
      physics::LinkPtr left_finger = model->GetLink("L2");
      physics::LinkPtr right_finger = model->GetLink("L1");
      physics::CollisionPtr col_left = left_finger->GetCollision("L2_collision");
      physics::CollisionPtr col_right= right_finger->GetCollision("L1_collision");
      physics::SurfaceParamsPtr sur_left = col_left->GetSurface();
      physics::SurfaceParamsPtr sur_right = col_right->GetSurface();
      m_surface_left = sur_left->FrictionPyramid();
      m_surface_right = sur_right->FrictionPyramid();
      // gzdbg << m_surface_left->MuPrimary() << std::endl;
      m_surface_left->SetMuPrimary(1e3);
      // gzdbg << m_surface_left->MuPrimary() << std::endl;
      m_surface_right->SetMuPrimary(1e5);
      m_surface_left->SetMuSecondary(1e5);
      m_surface_right->SetMuSecondary(1e5);
      gzdbg << "Loaded" << std::endl;
      // this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));

      // New method
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "gazebo_client",
            ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<gripper_controls::Friction>(
            "/" + this->model->GetName() + "/friction_setter",
            1,
            boost::bind(&ModelPush::OnRosMsg, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);

      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelPush::QueueThread, this));
    }

    void SetFriction(int left, int right){
      if (left==1){
          m_surface_left->SetMuPrimary(1e5);
          m_surface_left->SetMuSecondary(1e5);
      }
      else{
          m_surface_left->SetMuPrimary(1e-1);
          m_surface_left->SetMuSecondary(1e-1);
      }
      gzdbg << m_surface_left->MuPrimary() << std::endl;
      if (right==1){
          m_surface_right->SetMuPrimary(1e5);
          m_surface_right->SetMuSecondary(1e5);
      }
      else{
          m_surface_right->SetMuPrimary(1e-1);
          m_surface_right->SetMuSecondary(1e-1);
      }
    }

    public: void OnRosMsg(const gripper_controls::FrictionConstPtr &_msg)
    {

      this->SetFriction(_msg->left_friction,_msg->right_friction);
    }

  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    physics::FrictionPyramidPtr m_surface_left;
    physics::FrictionPyramidPtr m_surface_right;

    /// \brief A node use for ROS transport
    std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    std::thread rosQueueThread;

    void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
  };

  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
