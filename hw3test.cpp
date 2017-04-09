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

#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace std;

const int MAX_TTURTLES = 3;
const int MAX_XTURTLES = 3;
const double DANGER_TOLERANCE = 0.5;
const double LOWER_LIMIT = 0.0;
const double UPPER_LIMIT = 11.0;

int killed = 0;

struct TurtlePose {
  string turtlename;
  string topicname;
  turtlesim::Pose pose;
};

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
  static bool closeToKill(double x1, double y1, double x2, double y2, double threshhold);
  static void removeTurtle1();
  static void killTurtle();
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


/*makes sure no overlap*/
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

bool closeToKill(double x1, double y1, double x2, double y2, double threshhold) {
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



void killTurtle(int turtleIdt) {
  turtlesim::Kill::Request reqk;
  turtlesim::Kill::Response respk;

  //HW::tturtles[i].turtlename = req.name;
  reqk.name = HW::tturtles[turtleIdt].turtlename;
  //reqk.name = HW::turtle1.turtlename;
  if (!HW::kClient.call(reqk, respk))
     ROS_ERROR_STREAM("Error: Failed to kill " << reqk.name.c_str() << "\n");
  //else
     //ROS_INFO_STREAM("!!! Mission failed !!!");
  else
  {
    killed++;

    if (killed == MAX_TTURTLES)
    {
       ROS_INFO_STREAM("Congratulations Turtle1 Captured all the Targets");
       ROS_INFO_STREAM("...shutting down...\n");
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
    bool closeToKill();

    //added
    void moveTurtle();
};

//turtle1 callback
void Turtle1Listener::doTest(const turtlesim::Pose::ConstPtr& msg) {
  //update turtle1 pose whenever turtle1 moves
  HW::turtle1.pose.x = msg->x;
  HW::turtle1.pose.y = msg->y;

  //test case1
  if (isOffBoundary())
     HW::removeTurtle1();

  //test case2
  if (isTooClose())
     HW::removeTurtle1();

   //cout << "(" << HW::turtle1.pose.x << "," << HW::turtle1.pose.y << ")" << endl;

   moveTurtle();
     
  //test case3
  //if (closeToKill())
  //   HW::killTurtle(); 
}; 

/* turtle1 has left the boundry & have failed the mission */
bool Turtle1Listener::isOffBoundary() {
    //will have to change for the sake of having the turtle at 0,0
  if (HW::turtle1.pose.x < LOWER_LIMIT || HW::turtle1.pose.x > UPPER_LIMIT || HW::turtle1.pose.y < LOWER_LIMIT || HW::turtle1.pose.y > UPPER_LIMIT) {
     ROS_INFO_STREAM("turtle1 is moving off the limit at (" << HW::turtle1.pose.x << "," << HW::turtle1.pose.y << ")");
     return true;
        system("rosservice call /kill turtle1");

  } else 
     return false;
}

/* turtle1 has gotten too close to the enemy turtles & have failed the mission */
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


/* turtle1 has Captured the target */
bool Turtle1Listener::closeToKill() {
  int i;
  bool toCapture = false;
  double dist;

  //when turtle1 moves, check all T turtles' locations
  for (i=0; i<MAX_TTURTLES; i++) {
     dist = HW::getDistance(HW::turtle1.pose.x, HW::turtle1.pose.y, HW::tturtles[i].pose.x, HW::tturtles[i].pose.y);
     if (dist <= DANGER_TOLERANCE) {
        toCapture = true;
        ROS_INFO_STREAM("Captured " << HW::tturtles[i].turtlename << " with distance = " << dist);

        break;
     };
  };       
  return toCapture; 
}

//ADDED
void Turtle1Listener::moveTurtle()
{
  ros::NodeHandle nh;

  //ros::Publisher pub = HWTest::_nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 1000);
  geometry_msgs::Twist msg;
  int closestTurtle = 0;

  double distanceToTurtle = 100000;
  double tempDistance = 0;

    for(int i = 0; i < MAX_TTURTLES; i++)
    {
      tempDistance = HW::getDistance(HW::tturtles[i].pose.x, HW::tturtles[i].pose.y, HW::turtle1.pose.x, HW::turtle1.pose.y);
      //cout << "Distance to turtle[" << i << "]: " << distanceToTurtle << endl;
      //cout << "tempDistance: " << tempDistance << endl;
      if (tempDistance < distanceToTurtle)
      {
        distanceToTurtle = tempDistance;
        closestTurtle = i;
      }
      //cout << "Closest Turtle is turtle[" << closestTurtle << "]" << endl << endl;;
    }
    
    msg.linear.x = 1;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = atan(((HW::tturtles[closestTurtle].pose.y - HW::turtle1.pose.y) / (HW::tturtles[closestTurtle].pose.x - HW::turtle1.pose.x) - HW::turtle1.pose.theta));

    cout << "Moving towards : (" << HW::tturtles[closestTurtle].pose.x << " , " << HW::tturtles[closestTurtle].pose.y << ")" << endl;
    cout << "Current Position: (" << HW::turtle1.pose.x << " , " << HW::turtle1.pose.y << ")" << endl;
    cout << "Angular z: " << msg.angular.z << endl;
    
    pub.publish(msg);

}

class XTurtleListener {
  public: 
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename); 
  private:
    bool isTooClose(int ti);
};

class TTurtleListener {
  public: 
    void doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename); 
  private:
    bool closeToKill(int ti);
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

void TTurtleListener::doTest(const turtlesim::Pose::ConstPtr& msg, const string turtlename) {
int turtleIdt;
  //update a tturtle pose whenever tturtle moves
  turtleIdt = atoi(turtlename.substr(1).c_str()); //extract turtle # from turtlename
  turtleIdt = turtleIdt - 1; //since index starts from 0
  HW::tturtles[turtleIdt].pose.x = msg->x;
  HW::tturtles[turtleIdt].pose.y = msg->y;
   if (closeToKill(turtleIdt)) {
     HW::killTurtle(turtleIdt);
  };
 }

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

bool TTurtleListener::closeToKill(int ti) {
  double dist;
  bool tookill = false;

  dist = HW::getDistance(HW::tturtles[ti].pose.x, HW::tturtles[ti].pose.y, HW::turtle1.pose.x, HW::turtle1.pose.y);
  if (dist <= DANGER_TOLERANCE) { 
     tookill = true;
     ROS_INFO_STREAM("Turtle1 captured " << HW::tturtles[ti].turtlename << " with distance = " << dist);
  };
  return tookill;
}

//CLASS HWTest DEFINITIONS
class HWTest {
  public: 
    HWTest(ros::NodeHandle* anh) {
      _nh = *anh;
    };

    void init();
    void startTest();

    //ADDED
    //void moveTurtle();

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
  
  //kill starter turtle1
  system("rosservice call /kill turtle1");
  system("rosservice call /spawn 0 11 0 turtle1");

  HW::sClient = _nh.serviceClient<turtlesim::Spawn>("spawn");
  HW::kClient = _nh.serviceClient<turtlesim::Kill>("kill");

  //set seed for random number
  srand(time(0));

  HW::turtle1.turtlename = "turtle1";
  HW::turtle1.topicname = "/turtle1/pose";

  //create T turtles before X turtles to avoid landing xturtle on the same location
  HW::createTurtles('T', MAX_TTURTLES);
  HW::createTurtles('X', MAX_XTURTLES);

  //create turtle1 subscriber 
  _turtle1sub = _nh.subscribe<turtlesim::Pose>(HW::turtle1.topicname, 1000, &Turtle1Listener::doTest, &_turtle1listener); 
  //create xturtle subscribers 
  for (i=0; i<MAX_XTURTLES; i++) {
     _xturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::xturtles[i].topicname, 1000, boost::bind(&XTurtleListener::doTest, &_xturtlelisteners[i], _1, HW::xturtles[i].turtlename));
  };
//create tturtle subscribers 
  for (i=0; i<MAX_TTURTLES; i++) {
     _tturtlesubs[i] = _nh.subscribe<turtlesim::Pose>(HW::tturtles[i].topicname, 1000, boost::bind(&TTurtleListener::doTest, &_tturtlelisteners[i], _1, HW::tturtles[i].turtlename));
  };
}

void HWTest::startTest() {
  ROS_INFO_STREAM("---------------- Ready to Test ----------------");
  ROS_INFO_STREAM("1. turtle1 will be removed if it moves off the limit (" 
                  << LOWER_LIMIT << "," 
                  << LOWER_LIMIT << ") and (" 
                  << UPPER_LIMIT << "," 
                  << UPPER_LIMIT << ")");
  ROS_INFO_STREAM("2. turtle1 can capture T turtle within the distance " << DANGER_TOLERANCE);
  ROS_INFO_STREAM("3. X turtle will capture turtle1 within the distance " << DANGER_TOLERANCE);
  ROS_INFO_STREAM("-----------------------------------------------");

  ros::Rate loop_rate(1);
  
  /*do
  {
      moveTurtle();
      ros::spinOnce();
      loop_rate.sleep();
  }while (HW::turtleExist("turtle1") && (killed < MAX_TTURTLES));
  */
  ros::spin();
  }


int main(int argc, char **argv) {
  ros::init(argc, argv, "HWTest");
  ros::NodeHandle nh;

  
  //ros::Rate loopRate(2);

  HWTest hw3t(&nh);
  hw3t.init();
  hw3t.startTest();
  
  //loopRate.sleep();

  
  return 0;
}