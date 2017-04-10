#include "geometry_msgs/Twist.h"
#include <sstream>
#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <sstream>
#include <ros/master.h>
#include <boost/algorithm/string.hpp>
using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;

const int MAX_TTURTLES = 3;
const int MAX_XTURTLES = 4;
const double DANGER_TOLERANCE = 0.5;
const double LOWER_LIMIT = 0.0;
const double UPPER_LIMIT = 11.0;

int killed = 0;

const double x_min = 0.0;
const double y_min = 0.0;
const double x_max = 11.0;
const double y_max = 11.0;
const double PI = 3.14159265359;

struct TurtlePose {
  string turtlename;
  string topicname;
  turtlesim::Pose pose;
};

TurtlePose turtle1;
TurtlePose tturtles[MAX_TTURTLES];
TurtlePose xturtles[MAX_XTURTLES];

void move(double speed, double distance, bool isForward);

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

void rotate(double angular_speed, double relative_angle, bool clockwise);

double degrees2radians(double angle_in_degrees);

void setDesiredOrientation(double desired_angle_radians);

void moveGoal (turtlesim::Pose goal_pose, double distance_tolerance);

double getDistance(double x1, double y1, double x2, double y2);

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_move");
    turtlesim::Pose goal_pose;
    ros::NodeHandle n;
    ros::Subscriber _xturtlesubs[MAX_XTURTLES];
    ros::Subscriber _tturtlesubs[MAX_TTURTLES];

    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    // for (i=0; i<MAX_XTURTLES; i++) {
    //    _xturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::xturtles[i].topicname, 1000, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[i], _1, HW::xturtles[i].turtlename));
    // };
    //
    // for (i=0; i<MAX_TTURTLES; i++) {
    //    _tturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::tturtles[i].topicname, 1000, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[i], _1, HW::xturtles[i].turtlename));
    // };
    ros::Rate loop_rate(10);
    ROS_INFO_STREAM("\n\n\n******START MOVING******\n");
    goal_pose.x = 2.0;
    goal_pose.y = 11.0;
    goal_pose.theta = 0;
    moveGoal(goal_pose, 0.01);
    loop_rate.sleep();
    // speed = 0.2;
    // distance = 10;
    // isForward = 1;
    // move(speed, distance, isForward);
    ros::spin();
    return 0;
}

void move(double speed, double distance, bool isForward) {
    //distance = speed*time;
    geometry_msgs::Twist vel_msg;

    if(isForward) {
        vel_msg.linear.x = abs(speed);
    } else {
        vel_msg.linear.x = -abs(speed);
    }

    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;
    vel_msg.angular.z = 0;

    double t0 = ros::Time::now().toSec();
    double current_distance = 0.0;
    ros::Rate loop_rate(100);

    double t1;

    do {
        velocity_publisher.publish(vel_msg);
        t1 = ros::Time::now().toSec();
        current_distance = speed*(t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    } while(current_distance < distance);

    vel_msg.linear.x = 0;
    velocity_publisher.publish(vel_msg);
}

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message) {
    turtle1.pose.x = pose_message->x;
    turtle1.pose.y = pose_message->y;
    turtle1.pose.theta = pose_message->theta;
}

void rotate(double angular_speed, double relative_angle, bool clockwise) {
    geometry_msgs::Twist vel_msg;

    vel_msg.linear.x = 0;
    vel_msg.linear.y = 0;
    vel_msg.linear.z = 0;

    vel_msg.angular.x = 0;
    vel_msg.angular.y = 0;

    if(clockwise) {
        vel_msg.angular.z = -abs(angular_speed);
    } else {
        vel_msg.angular.z = abs(angular_speed);
    }

    double current_angle = 0.0;

    double t0 = ros::Time::now().toSec();
    ros::Rate loop_rate(10);

    do {
        velocity_publisher.publish(vel_msg);
        double t1 = ros::Time::now().toSec();
        current_angle = angular_speed * (t1-t0);
        ros::spinOnce();
        loop_rate.sleep();
    }while(current_angle < relative_angle);

    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

double degrees2radians(double angle_in_degrees) {
    return angle_in_degrees * PI / 180.0;
}

void setDesiredOrientation (double desired_angle_radians) {
    double relative_angle_radians = desired_angle_radians - turtle1.pose.theta;
    bool clockwise = ((relative_angle_radians)?true:false);
    rotate (degrees2radians(10), abs(relative_angle_radians), clockwise);
}

void moveGoal (turtlesim::Pose goal_pose, double distance_tolerance) {
    geometry_msgs::Twist vel_msg;
    ros::Rate loop_rate(100);
    double E = 0.0;

    do {
        //PID
        double Kp = 1.0;
        double e = getDistance(turtle1.pose.x, turtle1.pose.y, goal_pose.x, goal_pose.y);
        double E = E + e;

        vel_msg.linear.x = (Kp * e);
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        double Kh = 1.5;
        vel_msg.angular.z = Kh*(atan2(goal_pose.y - turtle1.pose.y, goal_pose.x - turtle1.pose.x) - turtle1.pose.theta);
        velocity_publisher.publish(vel_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }while(getDistance(turtle1.pose.x, turtle1.pose.y, goal_pose.x, goal_pose.y) > distance_tolerance);

    cout << "end move goal" << endl;
    vel_msg.linear.x = 0;
    vel_msg.angular.z = 0;
    velocity_publisher.publish(vel_msg);
}

double getDistance(double x1, double y1, double x2, double y2) {
    return sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));
}
