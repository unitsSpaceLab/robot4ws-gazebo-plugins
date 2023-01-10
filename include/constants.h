#include <string>
#include <stdio.h>

double wheel_radius = 0.085;

using namespace std;


struct steer_geom
{
    double alpha;
    double L;
    double R;
    double driving_mode;
};

struct fake_publishers_config
{
    bool enable;
    string motorCurrentTopicName;
    string motorTemperatureTopicName;
    string soundTopicName;
    string jointStateTopicName;
    string sensorStateTopicName;
};


struct odom_config
{
    int odom_que_size;
    string odom_topic_name;
    string odom_frame_name;
    string base_link_frame_name;
    double odom_publish_rate;
    fake_publishers_config fake_publishers;
    
};


struct joint_pair
{
    /* data */
    int id;

    string steer_joint;
    string drive_joint;
    steer_geom geom;

};


