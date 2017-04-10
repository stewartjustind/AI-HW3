// This program can be used to your assignment#3.
// Run this program before your program is started.
#include <ros/ros.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <turtlesim/Pose.h>
#include <stdlib.h>
#include <sstream>
#include <ros/master.h>
#include <boost/algorithm/string.hpp>

#include "geometry_msgs/Twist.h"

using namespace std;

ros::Publisher velocity_publisher;
ros::Subscriber pose_subscriber;
ros::Publisher turtle1_pose_pub;

turtlesim::Pose turtlesim_pose;

/*ros::Subscriber _xturtlesubs[MAX_XTURTLES];
ros::Subscriber _tturtlesubs[MAX_TTURTLES];
XTurtleListener _xturtlelisteners[MAX_XTURTLES];
TTurtleListener _tturtlelisteners[MAX_TTURTLES];*/

const int MAX_TTURTLES = 3;
const int MAX_XTURTLES = 4;
const double DANGER_TOLERANCE = 0.5;
const double LOWER_LIMIT = 0.0;
const double UPPER_LIMIT = 11.0;
const double PI = 3.14159265359;

int killed = 0;

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

//methods to move the robot straight
void move(double speed, double distance, bool isForward);
void movey(double speed, double distance, bool isForward);

//method to make the turtle turn
void rotate (double angular_speed, double angle, bool clockwise);
double degrees2radians(double angle_in_degrees);

//callback the position of the turtle
//enter into terminal: rostopic turtle1/pose
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message);

void setDesiredOrientation(double desired_angle_radians);

void moveGoal (turtlesim::Pose goal_pose, double distance_tolerance);


///////////////////////////////
namespace HW {
  static TurtlePose turtle1;
  static TurtlePose tturtles[MAX_TTURTLES];
  static TurtlePose xturtles[MAX_XTURTLES];
  static ros::ServiceClient sClient;
  static ros::ServiceClient kClient;
  static string getTurtlename(const string topicname);
  static bool topicExist(const string topicname);
  static bool turtleExist(const string turtlename);
  static turtlesim::Pose getNonOverlappingPoint(char tType);
  static void createTurtles(char tType, int cnt);
  static double getDistance(double x1, double y1, double x2, double y2);
  static bool isTooClose(double x1, double y1, double x2, double y2, double threshhold);
  static bool canCapture(double x1, double y1, double x2, double y2, double threshhold);
  static void removeTurtle1();
  static void captureTurtleT();
};

namespace HW {

string getTurtlename(const string topicname) {
  vector<string> elems;
  char lc_delim[2];
  lc_delim[0] = '/';
  lc_delim[1] = '\0';

  boost::algorithm::split(elems, topicname, boost::algorithm::is_any_of(lc_delim));
  return elems[1];
}

bool topicExist(const string topicname) {
  int i;
  string tname;
  ros::master::V_TopicInfo alltopics;

  //get all topic names
  ros::master::getTopics(alltopics);

  for (int i=0; i<alltopics.size(); i++) {
     tname = alltopics[i].name;
     if (tname.compare(topicname) == 0) {
        return true;
     };
  };
  return false;
}

bool turtleExist(const string turtlename) {
  int i;
  string tname;
  ros::master::V_TopicInfo alltopics;

  //get all topic names
  ros::master::getTopics(alltopics);

  for (int i=0; i<alltopics.size(); i++) {
     tname = getTurtlename(alltopics[i].name);
     if (tname.compare(turtlename) == 0) {
        return true;
     };
  };
  return false;
}

turtlesim::Pose getNonOverlappingPoint(char tType) {
  turtlesim::Pose xp;
  bool tooclose = false;
  int i;
  int ocnt=0;

  xp.x = double((rand() % 10) + 2.0);
  xp.y = double((rand() % 10) + 2.0);

  while (true) {
    if (HW::isTooClose(HW::turtle1.pose.x, HW::turtle1.pose.y, xp.x, xp.y, DANGER_TOLERANCE))
        tooclose = true;
    else if (tType == 'T')
            break; //out of while loop
    else { //X turtle needs to check all T turtles
       for (i=0; i<MAX_TTURTLES; i++) {
           if (HW::isTooClose(HW::tturtles[i].pose.x, HW::tturtles[i].pose.y, xp.x, xp.y, DANGER_TOLERANCE)) {
              tooclose = true;
              break; //out of for loop and regenerate a point
           };
       };
    };

    if (!tooclose) //checking for X turtle case
       break; //out of while loop

    if (ocnt>1000) { //only to check abnormality
       ROS_INFO_STREAM("chk: " << xp.x << "," << xp.y << "\n");
       break; //possibly wrong so exit
    };
    //generate another random pose
    xp.x = double((rand() % 10) + 2.0);
    xp.y = double((rand() % 10) + 2.0);
    tooclose = false;
    ocnt++;
    ROS_INFO_STREAM(".");
  };
  return xp;
}

void createTurtles(char tType, int cnt) {
  int i;
  stringstream tname, cmdstr;
  bool success = false;
  turtlesim::Spawn::Request req;
  turtlesim::Spawn::Response resp;
  turtlesim::Pose nop;

  for (i=0; i<cnt; i++) {
     tname.clear();
     tname.str("");
     tname << tType << i + 1;
     req.name = tname.str();
     nop = HW::getNonOverlappingPoint(tType);
     req.x = nop.x;
     req.y = nop.y;
     req.theta = M_PI/2; //face up for target turtles

     tname.clear();
     tname.str("");
     tname << "/" << req.name << "/pose";

     //fill out turtles tables for pose tracking
     if (tType == 'X') {
        req.theta = 3.0*req.theta; //change to face down for villain turtles
        HW::xturtles[i].turtlename = req.name;
        HW::xturtles[i].topicname = tname.str();
        HW::xturtles[i].pose.x = req.x;
        HW::xturtles[i].pose.y = req.y;
        HW::xturtles[i].pose.theta = req.theta;
     }
     else {
        HW::tturtles[i].turtlename = req.name;
        HW::tturtles[i].topicname = tname.str();
        HW::tturtles[i].pose.x = req.x;
        HW::tturtles[i].pose.y = req.y;
        HW::tturtles[i].pose.theta = req.theta;
     };

     //if this turtle does not exist, create one else teleport it.
     if (!turtleExist(req.name.c_str())) {
        success = HW::sClient.call(req, resp);
        if(success) {
           if (tType == 'X')
              ROS_INFO("%s landed with face down.", req.name.c_str()); //X turtle
           else
              ROS_INFO("%s landed with face up.", req.name.c_str()); //T turtle
        }
        else {
          ROS_ERROR_STREAM("Error: Failed to create " << tType << " turtle.");
          ros::shutdown();
        }
     }
     else {
        cmdstr.clear();
        cmdstr.str("");
        cmdstr << "rosservice call /";
        cmdstr << req.name.c_str() << "/teleport_absolute " << req.x << " " << req.y << " " << req.theta;
        system(cmdstr.str().c_str());
        ROS_INFO_STREAM(req.name.c_str() << " already landed, so it's teleported!\n");
     };
  };
}

double getDistance(const double x1, const double y1, const double x2, const double y2) {
  return sqrt(pow((x1-x2),2) + pow(y1-y2, 2));
}

bool isTooClose(double x1, double y1, double x2, double y2, double threshhold) {
  if (HW::getDistance(x1, y1, x2, y2) <= threshhold)
     return true;
  else
     return false;
}

bool canCapture(double x1, double y1, double x2, double y2, double threshhold) {
  if (HW::getDistance(x1, y1, x2, y2) <= threshhold)
     return true;
  else
     return false;
}

void removeTurtle1() {
  turtlesim::Kill::Request reqk;
  turtlesim::Kill::Response respk;

  reqk.name = HW::turtle1.turtlename;
  if (!HW::kClient.call(reqk, respk))
     ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
  else
     ROS_INFO_STREAM("!!! Mission failed !!!");

  ROS_INFO_STREAM("...shutting down...\n");
  ros::shutdown();
}

void captureTurtleT() {
    turtlesim::Kill::Request reqk;
    turtlesim::Kill::Response respk;
    int i;
    double dist;

    for (i=0; i<MAX_TTURTLES; i++) {
       dist = HW::getDistance(HW::turtle1.pose.x, HW::turtle1.pose.y, HW::tturtles[i].pose.x, HW::tturtles[i].pose.y);
       if (dist <= DANGER_TOLERANCE) {
          reqk.name = HW::tturtles[i].turtlename;
          break;
       };
    };

    if(!HW::kClient.call(reqk, respk)) {
        ROS_ERROR_STREAM("Error: Failed to capture " <<  HW::tturtles[i].turtlename << "\n");

    } else {
        HW::tturtles[i].pose.x = -1;
        HW::tturtles[i].pose.y = -1;
        ROS_INFO_STREAM("!!! " <<  HW::tturtles[i].turtlename << " Captured !!!");
        killed++;
        if(killed == MAX_TTURTLES) {
            ROS_INFO_STREAM("!!! CONGRATULATIONS!! ALL TURTLES CAPTURED! !!!");
            ROS_INFO_STREAM("!!! MISSION ACCOMPLISHED !!!");
            ros::shutdown();
        }
    }

}
}; //end of namespace
///////////////////////////////////////

class Turtle1Listener {
  public:
    void doTest(const turtlesim::Pose::ConstPtr& msg);
  private:
    bool isOffBoundary();
    bool isTooClose();
    bool canCapture();
};

//turtle1 callback
void Turtle1Listener::doTest(const turtlesim::Pose::ConstPtr& msg) {
    //update turtle1 pose whenever turtle1 moves
    HW::turtle1.pose.x = msg->x;
    HW::turtle1.pose.y = msg->y;
    string tTurtleName = "";
    //test case1
    if (isOffBoundary())
        HW::removeTurtle1();

    //test case2
    if (isTooClose())
        HW::removeTurtle1();

    //test case 3
    if(canCapture())
        HW::captureTurtleT();
};

bool Turtle1Listener::isOffBoundary() {
  if (HW::turtle1.pose.x < LOWER_LIMIT || HW::turtle1.pose.x > UPPER_LIMIT || HW::turtle1.pose.y < LOWER_LIMIT || HW::turtle1.pose.y > UPPER_LIMIT) {
     ROS_INFO_STREAM("turtle1 is moving off the limit at (" << HW::turtle1.pose.x << "," << HW::turtle1.pose.y << ")");
     return true;
  } else
     return false;
}

bool Turtle1Listener::isTooClose() {
  int i;
  bool tooclose = false;
  double dist;

  //when turtle1 moves, check all X turtles' locations
  for (i=0; i<MAX_XTURTLES; i++) {
     dist = HW::getDistance(HW::turtle1.pose.x, HW::turtle1.pose.y, HW::xturtles[i].pose.x, HW::xturtles[i].pose.y);
     if (dist <= DANGER_TOLERANCE) {
        tooclose = true;
        ROS_INFO_STREAM("Turtle1 was too close to " << HW::xturtles[i].turtlename << " with distance = " << dist);
        ROS_INFO_STREAM("turtle1 was captured.");
        break;
     };
  };
  return tooclose;
}

bool Turtle1Listener::canCapture() {

  int i;
  bool canCapture = false;
  double dist;

  //when turtle1 moves, check all T turtles' locations
  for (i=0; i<MAX_TTURTLES; i++) {
     dist = HW::getDistance(HW::turtle1.pose.x, HW::turtle1.pose.y, HW::tturtles[i].pose.x, HW::tturtles[i].pose.y);
     if (dist <= DANGER_TOLERANCE) {
        canCapture = true;
        ROS_INFO_STREAM(HW::tturtles[i].turtlename << " was captured!");
        break;
     };
  };
  return canCapture;
}

class XTurtleListener {
  public:
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename);
  private:
    bool isTooClose(int ti);
};

//xturtle callback
void XTurtleListener::doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename) {
  int turtleIdx;
  //update a xturtle pose whenever xturtle moves
  turtleIdx = atoi(turtlename.substr(1).c_str()); //extract turtle # from turtlename
  turtleIdx = turtleIdx - 1; //since index starts from 0
  HW::xturtles[turtleIdx].pose.x = msg->x;
  HW::xturtles[turtleIdx].pose.y = msg->y;

  if (isTooClose(turtleIdx)) {
     HW::removeTurtle1();
  };

};

//when a xturtle moves, check the turtle1's location
bool XTurtleListener::isTooClose(int ti) {
  double dist;
  bool tooclose = false;

  dist = HW::getDistance(HW::xturtles[ti].pose.x, HW::xturtles[ti].pose.y, HW::turtle1.pose.x, HW::turtle1.pose.y);
  if (dist <= DANGER_TOLERANCE) {
     tooclose = true;
     ROS_INFO_STREAM("Turtle1 was too close to " << HW::xturtles[ti].turtlename << " with distance = " << dist);
     ROS_INFO_STREAM("turtle1 was captured.");
  };
  return tooclose;
}

class TTurtleListener {
  public:
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename);
  private:
    bool canCapture(int ti);
};

//tturtle callback
void TTurtleListener::doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename) {

  int turtleIdt;
  //update a tturtle pose whenever tturtle moves
  turtleIdt = atoi(turtlename.substr(1).c_str()); //extract turtle # from turtlename
  turtleIdt = turtleIdt - 1; //since index starts from 0
  HW::tturtles[turtleIdt].pose.x = msg->x;
  HW::tturtles[turtleIdt].pose.y = msg->y;

  if (canCapture(turtleIdt)) {
     HW::captureTurtleT();
  };

};

bool TTurtleListener::canCapture(int ti) {
  double dist;
  bool canCapture = false;

  dist = HW::getDistance(HW::tturtles[ti].pose.x, HW::tturtles[ti].pose.y, HW::turtle1.pose.x, HW::turtle1.pose.y);
  if (dist <= DANGER_TOLERANCE) {
     canCapture = true;
     ROS_INFO_STREAM(HW::tturtles[ti].turtlename << " was captured!");
  };
  return canCapture;
}

class HWTest {
  public:
    HWTest(ros::NodeHandle* anh) {
      _nh = *anh;
    };

    void init();
    void startTest();

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _turtle1sub;
    ros::Subscriber _xturtlesubs[MAX_XTURTLES];
    ros::Subscriber _tturtlesubs[MAX_TTURTLES];
    Turtle1Listener _turtle1listener;
    XTurtleListener _xturtlelisteners[MAX_XTURTLES];
    TTurtleListener _tturtlelisteners[MAX_TTURTLES];
};

void HWTest::init() {
  int i;
  stringstream cmdstr;

  HW::sClient = _nh.serviceClient<turtlesim::Spawn>("spawn");
  HW::kClient = _nh.serviceClient<turtlesim::Kill>("kill");

  //set seed for random number
  srand(time(0));

  HW::turtle1.turtlename = "turtle1";
  HW::turtle1.topicname = "/turtle1/pose";

  //create T turtles before X turtles to avoid landing xturtle on the same location
  HW::createTurtles('T', MAX_TTURTLES);
  HW::createTurtles('X', MAX_XTURTLES);

  //create turtle1 subsriber
  _turtle1sub = _nh.subscribe<turtlesim::Pose>(HW::turtle1.topicname, 1000, &Turtle1Listener::doTest, &_turtle1listener);
  //create xturtle subsribers
  for (i=0; i<MAX_XTURTLES; i++) {
     _xturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::xturtles[i].topicname, 1000, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[i], _1, HW::xturtles[i].turtlename));
  };
  system("rosservice call /turtle1/teleport_absolute 0 0 0");
  system("rosservice call clear");
  for (i=0; i<MAX_TTURTLES; i++) {
      _tturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::tturtles[i].topicname, 1000, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[i], _1, HW::tturtles[i].turtlename));
  }
}

void HWTest::startTest() {
  ROS_INFO_STREAM("---------------- Ready to Test ----------------");
  ROS_INFO_STREAM("1. turtle1 will be removed if it moves off the limit (" << LOWER_LIMIT << "," << LOWER_LIMIT << ") and (" << UPPER_LIMIT << "," << UPPER_LIMIT << ")");
  ROS_INFO_STREAM("2. turtle1 can capture T turtle within the distance " << DANGER_TOLERANCE);
  ROS_INFO_STREAM("3. X turtle will capture turtle1 within the distance " << DANGER_TOLERANCE);
  ROS_INFO_STREAM("-----------------------------------------------");

  ros::spin();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "HWTest");
  ros::NodeHandle nh;
  //ros::Rate loopRate(2);
  
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


    /*_tturtlesubs[0] = n.subscribe<turtlesim::Pose>("/T1/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[0], _1, "T1"));
    _tturtlesubs[1] = n.subscribe<turtlesim::Pose>("/T2/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[1], _1, "T2"));
    _tturtlesubs[2] = n.subscribe<turtlesim::Pose>("/T3/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[2], _1, "T3"));
    _tturtlesubs[3] = n.subscribe<turtlesim::Pose>("/T4/pose", 10, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[3], _1, "T4"));

    _xturtlesubs[0] = n.subscribe<turtlesim::Pose>("/X1/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[0], _1, "X1"));
    _xturtlesubs[1] = n.subscribe<turtlesim::Pose>("/X2/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[1], _1, "X2"));
    _xturtlesubs[2] = n.subscribe<turtlesim::Pose>("/X3/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[2], _1, "X3"));
    _xturtlesubs[3] = n.subscribe<turtlesim::Pose>("/X4/pose", 10, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[3], _1, "X4"));*/

    ros::Rate loop_rate(10);

    ROS_INFO_STREAM("\n\n\n******START MOVING******\n");

    // goal_pose.x = tturtles[0].pose.x;
    // goal_pose.y = tturtles[0].pose.y;
    // goal_pose.theta = tturtles[0].pose.theta;
    // moveGoal(tturtles[1].pose, 0.01);
    loop_rate.sleep();
    ros::spin();


  HWTest hw3t(&nh);
  hw3t.init();
  hw3t.startTest();
  //loopRate.sleep();
  return 0;
}



//Move turtle1
//////////////////////////////////////////////////////////////////////////

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
