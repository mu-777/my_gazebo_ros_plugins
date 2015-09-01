#ifndef GAZEBO_ROS_P3DTF_HH
#define GAZEBO_ROS_P3DTF_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

namespace gazebo
{
  class GazeboRosP3DTF : public ModelPlugin
  {
    public:
      GazeboRosP3DTF();
      virtual ~GazeboRosP3DTF();
      void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    protected:
      virtual void UpdateChild();
      virtual void FiniChild();

    private:
      physics::WorldPtr world_;
      physics::ModelPtr model_;
      physics::LinkPtr target_link_;
      physics::LinkPtr reference_link_;
      std::string robot_namespace_;
      std::string link_name_;
      std::string reference_link_name_;
      std::string frame_name_;
      std::string child_frame_name_;
      std::string node_name_;
      math::Pose offset_;

      ros::NodeHandle* rosnode_;
      geometry_msgs::TransformStamped tf_stamped_;
      tf2_ros::TransformBroadcaster tf_broadcaster_;

      double update_rate_hz_;
      common::Time last_time_;
      void spin();
      boost::thread *spinner_thread_;

      // Pointer to the update event connection
      event::ConnectionPtr update_connection_;

      template <class T>
      T getSDFElement(sdf::ElementPtr _sdf, const std::string& element_name, const T& default_element) {
          if (_sdf->HasElement(element_name)){
              return _sdf->GetElement(element_name)->Get<T>();
          } else {
              return default_element;
          }
      }
  };
}
#endif
