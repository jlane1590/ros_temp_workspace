#include <pluginlib/class_loader.h>
#include <rsptilt_ik_plugin/ik_base.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl/frames.hpp>
#include <ros/ros.h>
#include <vector>

int main(int argc, char** argv)
{
  pluginlib::ClassLoader<ik_base::IKPlugin> ik_loader("rsptilt_ik_plugin", "ik_base::IKPlugin");

  try
  {
    const std::vector<double> min_angles (7,-1.5707);
    const std::vector<double> max_angles (7,1.5707);
    boost::shared_ptr<ik_base::IKPlugin> ik = ik_loader.createInstance("rsptilt_kinematics::InverseKinematics");
    ik->initialize(min_angles, max_angles);

    geometry_msgs::Pose target_pose1;
    target_pose1.orientation = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, 0.3);
    target_pose1.position.x = 0.0;
    target_pose1.position.y = 0.0;
    target_pose1.position.z = 0.015;

    // Convert the ROS pose to a KDL frame
    KDL::Frame frame;
    tf::poseMsgToKDL(target_pose1, frame);

    // Convert the seed array to a KDL joint array
    //KDL::JntArray seed = configurationStdToKdl(ik_seed_state);

    // Calculate the inverse position kinematics
    //std::vector<KDL::JntArray> kdl_solutions;
    //int res = ik_->CartToJnt(seed, frame, kdl_solutions);
    std::vector<double> joint_solution;
    joint_solution.resize(7);
    int res = ik->ik(frame, joint_solution);

    if(res<=0){
      ROS_INFO("IK Failed");
    }
    else{
      //ROS_INFO("IK Joint Solution");
      //for(unsigned int i = 0; i < 7; i++){
      //  ROS_INFO("Joint %d: %f", i, joint_solution[i]);
      //}
    }
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
  }

  return 0;
}