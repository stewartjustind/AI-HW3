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
ros::Publisher turtle1_pose_pub;

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
  double distanceToPlayer;
  turtlesim::Pose pose;
};

TurtlePose turtle1;
TurtlePose tturtles[MAX_TTURTLES];
TurtlePose xturtles[MAX_XTURTLES];

double getDistance(double x1, double y1, double x2, double y2);

class XTurtleListener {
  public:
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename);
  private:
};

//xturtle callback
void XTurtleListener::doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename) {
    int turtleIdt;
    //update a tturtle pose whenever tturtle moves
    turtleIdt = atoi(turtlename.substr(1).c_str()); //extract turtle # from turtlename
    turtleIdt = turtleIdt - 1; //since index starts from 0
    xturtles[turtleIdt].pose.x = msg->x;
    xturtles[turtleIdt].pose.y = msg->y;
    xturtles[turtleIdt].distanceToPlayer = getDistance(turtle1.pose.x, turtle1.pose.y, msg->x, msg->y);
};

class TTurtleListener {
  public:
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename);
  private:
};

//tturtle callback
void TTurtleListener::doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename) {
    int turtleIdt;
    //update a tturtle pose whenever tturtle moves
    turtleIdt = atoi(turtlename.substr(1).c_str()); //extract turtle # from turtlename
    turtleIdt = turtleIdt - 1; //since index starts from 0
    tturtles[turtleIdt].pose.x = msg->x;
    tturtles[turtleIdt].pose.y = msg->y;
    tturtles[turtleIdt].distanceToPlayer = getDistance(turtle1.pose.x, turtle1.pose.y, msg->x, msg->y);
};

void move(double speed, double distance, bool isForward);

void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

void rotate(double angular_speed, double relative_angle, bool clockwise);

double degrees2radians(double angle_in_degrees);

void setDesiredOrientation(double desired_angle_radians);

void moveGoal (turtlesim::Pose goal_pose, double distance_tolerance);

ros::Subscriber _xturtlesubs[MAX_XTURTLES];
ros::Subscriber _tturtlesubs[MAX_TTURTLES];
XTurtleListener _xturtlelisteners[MAX_XTURTLES];
TTurtleListener _tturtlelisteners[MAX_TTURTLES];

int main(int argc, char **argv) {
    ros::init(argc, argv, "turtle_move");
    turtlesim::Pose goal_pose;
    ros::NodeHandle n;
    double speed, angular_speed;
    double distance, angle;
    bool isForward, clockwise;
    int i;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);
    pose_subscriber = n.subscribe("/turtle1/pose", 10, poseCallback);

    //ADDED
    turtle1_pose_pub = n.advertise<turtlesim::Pose>("/turtle1/pose", 1000);


    _tturtlesubs[0] = n.subscribe<turtlesim::Pose>("/T1/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[0], _1, "T1"));
    _tturtlesubs[1] = n.subscribe<turtlesim::Pose>("/T2/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[1], _1, "T2"));
    _tturtlesubs[2] = n.subscribe<turtlesim::Pose>("/T3/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[2], _1, "T3"));
    _tturtlesubs[3] = n.subscribe<turtlesim::Pose>("/T4/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[3], _1, "T4"));

    _xturtlesubs[0] = n.subscribe<turtlesim::Pose>("/X1/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[0], _1, "X1"));
    _xturtlesubs[1] = n.subscribe<turtlesim::Pose>("/X2/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[1], _1, "X2"));
    _xturtlesubs[2] = n.subscribe<turtlesim::Pose>("/X3/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[2], _1, "X3"));
    _xturtlesubs[3] = n.subscribe<turtlesim::Pose>("/X4/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[3], _1, "X4"));

    ros::Rate loop_rate(10);

    ROS_INFO_STREAM("\n\n\n******START MOVING******\n");

    // goal_pose.x = tturtles[0].pose.x;
    // goal_pose.y = tturtles[0].pose.y;
    // goal_pose.theta = tturtles[0].pose.theta;
    // moveGoal(tturtles[1].pose, 0.01);
    loop_rate.sleep();
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

    ros::Rate loop_rate(10);

    //moveGoal(tturtles[1].pose, 0.01);

    int closestTurtle = 0;
    double shortestDistance = 10000;

    for (int i = 0; i < MAX_TTURTLES; i++)
    {
        if ( tturtles[i].distanceToPlayer < shortestDistance)
            closestTurtle = i;
    }
    cout << "going to " << tturtles[closestTurtle].turtlename << ": (" << tturtles[closestTurtle].pose.x << "," << tturtles[closestTurtle].pose.y << ")" << endl;
    moveGoal(tturtles[closestTurtle].pose, 0.01);
    loop_rate.sleep();
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

        turtle1_pose_pub.publish(turtle1.pose);

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

void getTurtles() {
    for (int i = 0; i < MAX_TTURTLES; i++) {

    }
}