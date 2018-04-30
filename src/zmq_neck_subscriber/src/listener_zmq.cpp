#include <ros/ros.h>
#include "std_msgs/Float64MultiArray.h"
#include <zeromq_cpp/zmq.hpp>

#include <pluginlib/class_loader.h>
#include <rsptilt_ik_plugin/ik_base.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>
#include <kdl/frames.hpp>
#include <vector>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener_zmq", ros::init_options::AnonymousName);
    ros::NodeHandle n;

    pluginlib::ClassLoader<ik_base::IKPlugin> ik_loader("rsptilt_ik_plugin", "ik_base::IKPlugin");

    const std::vector<double> min_angles (7,-1.5707);
    const std::vector<double> max_angles (7,1.5707);
    boost::shared_ptr<ik_base::IKPlugin> ik = ik_loader.createInstance("rsptilt_kinematics::InverseKinematics");
    ik->initialize(min_angles, max_angles);

    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);

    std::string TOPIC = "";
    subscriber.setsockopt(ZMQ_SUBSCRIBE, TOPIC.c_str(), TOPIC.length()); // allow all messages

    int linger = 0; // Proper shutdown ZeroMQ
    subscriber.setsockopt(ZMQ_LINGER, &linger, sizeof(linger));

    subscriber.connect("tcp://localhost:6666");

    ros::Publisher joint_cmd_pub = n.advertise<std_msgs::Float64MultiArray>("/joint_position_controller/command", 10);

    while (ros::ok())
    {
        zmq::message_t message;
        float x,y,z,r,p,w;
        int rc = subscriber.recv(&message);
        if (rc)
        {
            std::string recv_string;

            std::istringstream iss(static_cast<char*>(message.data()));
            iss >> x >> y >> z >> r >> p >> w;

            geometry_msgs::Pose target_pose;
            target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(r, p, w);
            target_pose.position.x = x;
            target_pose.position.y = y;
            target_pose.position.z = z;
            //ROS_INFO("%f %f %f %f %f %f",x,y,z,r,p,w);

            // Convert the ROS pose to a KDL frame
            KDL::Frame frame;
            tf::poseMsgToKDL(target_pose, frame);

            std::vector<double> joint_solution;
            joint_solution.resize(7);
            int res = ik->ik(frame, joint_solution);

            if(res<=0){
                ROS_INFO("IK Failed");
            }
            else{
                std_msgs::Float64MultiArray msg;

                msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
                msg.layout.dim[0].size = joint_solution.size();
                msg.layout.dim[0].stride = 1;
                msg.layout.dim[0].label = "command";

                msg.data.clear();
                msg.data.insert(msg.data.end(), joint_solution.begin(), joint_solution.end());

                joint_cmd_pub.publish(msg);
                ros::spinOnce();
            }
        }
    }

    // Clean up your socket and context here
    subscriber.close();
    context.close();
    return 0;
}