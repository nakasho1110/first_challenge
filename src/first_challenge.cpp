#include "first_challenge/first_challenge.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("init_y", init_y, {0});
    private_nh_.param("hz_", hz_, {10});
    private_nh_.param("is_straight", is_straight, {true});
    private_nh_.param("is_negative", is_negative, {false});
    sub_odom_ = nh_.subscribe("/roomba/odometry", 100, &FirstChallenge::odometry_callback, this);
    sub_laser_ = nh_.subscribe("/scan", 100, &FirstChallenge::laser_callback, this);
    pub_cmd_vel_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

void FirstChallenge::run()
{
    double r,p,y;
    tf::Quaternion quat(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(r, p, y);

    cmd_vel_.mode = 11;
    bool no_wall = show_scan();

    if(is_straight && no_wall){
        cmd_vel_.cntl.linear.x = 0.1;
        cmd_vel_.cntl.angular.z = 0;
        if(abs(init_odo - odometry_.pose.pose.position.x) > 1){
            is_straight = false;
            is_negative = false;
        }
    }else if(!is_straight && no_wall){
        cmd_vel_.cntl.linear.x = 0;
        cmd_vel_.cntl.angular.z = 1;
        std::cout << "==================" << std::endl;
        std::cout << "cur yaw : " << y << std::endl;
        std::cout << "cur init yaw : " << init_y << std::endl;
        if(init_y > 0 && y < 0){
            is_negative = true;
        }
        if(is_negative && y > 0){
            is_straight = true;
            init_odo = odometry_.pose.pose.position.x;
        }
        init_y = y;
    }else{
        cmd_vel_.cntl.linear.x = 0;
        cmd_vel_.cntl.angular.z = 0;
    }

    pub_cmd_vel_.publish(cmd_vel_);
}

void FirstChallenge::show_odom()
{
    // ROS_INFO_STREAM("odom: x: %f, y: %f, z: %f", odometry_.pose.pose.position.x, odometry_.pose.pose.position.y, odometry_.pose.pose.position.z);
    std::cout << "odom" << ": x:" << odometry_.pose.pose.position.x << " y:" <<  odometry_.pose.pose.position.y << " z:" <<  odometry_.pose.pose.position.z << std::endl;
}

bool FirstChallenge::show_scan()
{
    float range_min = 1e6;
    bool no_wall = true;
    // for (int i = 0; i < laser_.ranges.size(); i++) {
        // if (laser_.ranges[i] < range_min) {
            // range_min = laser_.ranges[i];
    //    }
    //}
    float sum = 0;
    int m_index = int(laser_.ranges.size() / 2);
    for(int i=m_index-5;i<=m_index+5;i++){
        sum += laser_.ranges[i];
    }
    range_min = sum / 11;

    if(range_min > 0.5){
        no_wall = true;
    }else{
        no_wall = false;
    }
    // ROS_INFO_STREAM("scan: min: %f", range_min);

    return no_wall;
}

void FirstChallenge::process()
{
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(laser_.ranges.size() > 0){
            run();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_challenge");
    FirstChallenge first_challenge;
    first_challenge.process();
    ros::spin();
    return 0;
}
