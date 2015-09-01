
#include <string>
#include <stdlib.h>
#include "my_gazebo_ros_p3dtf/my_gazebo_ros_p3dtf.h"

using namespace gazebo;

GazeboRosP3DTF::GazeboRosP3DTF(){}

GazeboRosP3DTF::~GazeboRosP3DTF(){
    delete rosnode_;
    delete spinner_thread_;
}
void GazeboRosP3DTF::FiniChild(){
    rosnode_->shutdown();
    spinner_thread_->join();
}

void GazeboRosP3DTF::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf){

    // Get the world name.
    this->world_ = _parent->GetWorld();
    this->model_ = _parent;

    // load parameters
    this->robot_namespace_ = getSDFElement(_sdf, "robotNamespace", std::string("")) + "/";
    this->node_name_ = getSDFElement(_sdf, "nodeName", std::string("p3dtf"));
    this->frame_name_ = getSDFElement(_sdf, "frameName", std::string("world"));
    this->child_frame_name_ = getSDFElement(_sdf, "childFrameName", std::string("localized_point"));
    this->offset_.pos = getSDFElement(_sdf, "xyzOffset", math::Vector3(0, 0, 0));
    this->offset_.rot = math::Quaternion(getSDFElement(_sdf, "rpyOffset", math::Vector3(0, 0, 0)));
    this->update_rate_hz_ = getSDFElement(_sdf, "updateRateHz", 100.0);
    this->reference_link_name_ = getSDFElement(_sdf, "referenceLinkName", std::string("origin"));
    this->link_name_ = getSDFElement(_sdf, "targetLinkName", std::string(""));

    this->target_link_ = _parent->GetLink(this->link_name_);
    if (!this->target_link_){
        ROS_FATAL("gazebo_ros_p3dtf plugin error: targetLinkName: %s does not exist\n",
                this->link_name_.c_str());
        return;
    }
    if (this->reference_link_name_ != "origin" && this->reference_link_name_ != "world" && this->reference_link_name_ != ""){
        this->reference_link_ = this->model_->GetLink(this->reference_link_name_);
    }
    if (!this->reference_link_){
        ROS_ERROR("gazebo_ros_p3dtf plugin: referenceLinkName: %s does not exist, will not publish pose\n",
                this->reference_link_name_.c_str());
        return;
    }

    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv,this->node_name_, ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
    this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

    this->last_time_ = this->world_->GetSimTime();
    this->tf_stamped_.header.frame_id = this->frame_name_;
    this->tf_stamped_.child_frame_id = this->child_frame_name_;

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->spinner_thread_ = new boost::thread( boost::bind( &GazeboRosP3DTF::spin, this) );
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&GazeboRosP3DTF::UpdateChild, this));
}

void GazeboRosP3DTF::UpdateChild(){

    // rate control
    common::Time cur_time = this->world_->GetSimTime();
    if (this->update_rate_hz_ > 0 && (cur_time - this->last_time_).Double() < (1.0/this->update_rate_hz_)) return;


    this->tf_stamped_.header.stamp.sec = cur_time.sec;
    this->tf_stamped_.header.stamp.nsec = cur_time.nsec;

    math::Pose pose = this->target_link_->GetWorldPose();

    if (this->reference_link_){
        math::Pose frame_pose = this->reference_link_->GetWorldPose();
        pose.pos = pose.pos - frame_pose.pos;
        pose.rot = frame_pose.rot.GetInverse() * pose.rot;
        //    pose.pos = frame_pose.rot.RotateVectorReverse(pose.pos);
        //    pose.rot *= frame_pose.rot.GetInverse();
    }

    pose.pos = pose.pos + this->offset_.pos;
    pose.rot = this->offset_.rot * pose.rot;
    pose.rot.Normalize();

    this->tf_stamped_.transform.translation.x = pose.pos.x;
    this->tf_stamped_.transform.translation.y = pose.pos.y;
    this->tf_stamped_.transform.translation.z = pose.pos.z;
    this->tf_stamped_.transform.rotation.x = pose.rot.x;
    this->tf_stamped_.transform.rotation.y = pose.rot.y;
    this->tf_stamped_.transform.rotation.z = pose.rot.z;
    this->tf_stamped_.transform.rotation.w = pose.rot.w;

    this->tf_broadcaster_.sendTransform(this->tf_stamped_);
    this->last_time_ = cur_time;
}


void GazeboRosP3DTF::spin(){
  while(ros::ok()) ros::spinOnce();
}


GZ_REGISTER_MODEL_PLUGIN(GazeboRosP3DTF);



























