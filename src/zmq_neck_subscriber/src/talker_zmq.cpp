#include <ros/ros.h>
#include <zeromq_cpp/zmq.hpp>
#include <math.h>
#include <stdio.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker_zmq", ros::init_options::AnonymousName);

    ros::Time::init(); // Workaround since we are not using NodeHandle
    int freq = 30;
    ros::Rate loop_rate(freq);

    //  Prepare our context and publisher
    zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:6666");

    int count = 0;
    const double M2PI = 6.28318530717958647693;
    while (ros::ok())
    {
        float x,y,z,r,p,w;
        x=0.0; y=0.0; z=0.0; p=0.0;
        //parametric equations for a circle of 15 degree amplitude
        w = 0.2618*cos((M2PI*count)/(4*freq));
        r = 0.2618*sin((M2PI*count)/(4*freq));
        if(++count >= (4*freq-1))
            count = 0;

        std::stringstream msg_str_stream;
        msg_str_stream << x << " " << y << " " << z << " " << r << " " << p << " " << w;
        std::string msg_str = msg_str_stream.str();
        zmq::message_t message(msg_str.length());
        memcpy(message.data(), msg_str.c_str(), msg_str.length());

        ROS_INFO_STREAM(msg_str);
        publisher.send(message);
        loop_rate.sleep();
    }
    // Clean up your socket and context here
    publisher.close();
    context.close();

    return 0;
}