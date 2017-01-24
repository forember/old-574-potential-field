// src/potential_field.cpp
#line 1194 "potential_field.lit"
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cstdlib>  // Needed for rand()
#include <ctime>    // Needed to seed random number generator with a time value
#include "tf/LinearMath/Quaternion.h" // Needed to convert rotation ...
#include "tf/LinearMath/Matrix3x3.h"  // ... quaternion into Euler angles

#include <ros/console.h>  // For rosout
// Additional Includes
#line 147 "potential_field.lit"
#include <cmath>
#line 978 "potential_field.lit"
#include <algorithm>  // For min

#line 1206 "potential_field.lit"


struct Pose {
  double x; // in simulated Stage units
  double y; // in simulated Stage units
  double heading; // in radians
  ros::Time t;    // last received time

  // Construct a default pose object with the time set to 1970-01-01
  Pose() : x(0), y(0), heading(0), t(0.0) {};

  // Process incoming pose message for current robot
  void poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    double roll, pitch;
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    tf::Quaternion q = tf::Quaternion(msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3(q).getRPY(roll, pitch, heading);
    t = msg->header.stamp;
  };
};


// Structs
#line 29 "potential_field.lit"
// Vector Struct
#line 39 "potential_field.lit"
struct Vector2 {
  double x;
  double y;

  // Vector Operator Overloads
  #line 48 "potential_field.lit"
  Vector2 &operator=(const Vector2 &v) {x=v.x; y=v.y; return *this;}
  Vector2 &operator+=(const Vector2 &v) {x+=v.x; y+=v.y; return *this;}
  const Vector2 operator+(const Vector2 &v) {return Vector2(*this)+=v;}
  Vector2 &operator-=(const Vector2 &v) {x-=v.x; y-=v.y; return *this;}
  const Vector2 operator-(const Vector2 &v) {return Vector2(*this)-=v;}
  Vector2 &operator=(const double &a) {x=y=a; return *this;}
  Vector2 &operator*=(const double &a) {x*=a; y*=a; return *this;}
  const Vector2 operator*(const double &a) {return Vector2(*this)*=a;}
  bool operator==(const Vector2 &v) {return x==v.x && y==v.y;}
  bool operator!=(const Vector2 &v) {return !(*this == v);}

#line 44 "potential_field.lit"
};

#line 30 "potential_field.lit"
#line 552 "potential_field.lit"
// Leader Struct
#line 569 "potential_field.lit"
struct Leader {
  double distanceThreshold;
  double thetaMin;
  double thetaMax;
};

#line 553 "potential_field.lit"

#line 1232 "potential_field.lit"


class PotFieldBot {
public:
  // Construst a new Potential Field controller object and hook up
  // this ROS node to the simulated robot's pose, velocity control,
  // and laser topics
  PotFieldBot(ros::NodeHandle& nh, int robotID, int n,  double gx, double gy)
      : ID(robotID), numRobots(n), goalX(gx), goalY(gy) {
    // Advertise a new publisher for the current simulated robot's
    // velocity command topic (the second argument indicates that
    // if multiple command messages are in the queue to be sent,
    // only the last command will be sent)
    commandPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Subscribe to the current simulated robot's laser scan topic and
    // tell ROS to call this->laserCallback() whenever a new message
    // is published on that topic
    laserSub = nh.subscribe("base_scan", 1, &PotFieldBot::laserCallback, this);

    // Subscribe to each robot' ground truth pose topic
    // and tell ROS to call pose->poseCallback(...) whenever a new
    // message is published on that topic
    for (int i = 0; i < numRobots; i++) {
      pose.push_back(Pose());
    }
    if (numRobots == 1) {
      poseSubs.push_back(nh.subscribe("/base_pose_ground_truth",
          1, &Pose::poseCallback, &pose[0]));
    } else {
      for (int i = 0; i < numRobots; i++) {
        poseSubs.push_back(nh.subscribe("/robot_" +
            boost::lexical_cast<std::string>(i) + "/base_pose_ground_truth",
            1, &Pose::poseCallback, &pose[i]));
      }
    }
  };


  // Send a velocity command
  void move(double linearVelMPS, double angularVelRadPS) {
    geometry_msgs::Twist msg;
          // The default constructor will set all commands to 0
    msg.linear.x = linearVelMPS;
    msg.angular.z = angularVelRadPS;
    commandPub.publish(msg);
  };


  // Process incoming laser scan message
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {

    // Laser Callback
    #line 520 "potential_field.lit"
    laserAngleMin = pose[ID].heading + msg->angle_min + M_PI;
    laserAngleIncrement = msg->angle_increment;
    #line 529 "potential_field.lit"
    laserRanges = &msg->ranges;
    #line 543 "potential_field.lit"
    laserMessage = msg;

#line 1285 "potential_field.lit"

  };


  // Main FSM loop for ensuring that ROS messages are
  // processed in a timely manner, and also for sending
  // velocity controls to the simulated robot based on the FSM state
  void spin() {
    ros::Rate rate(30); // Specify the FSM loop rate in Hz
    ROS_INFO("Entering spin.");

    while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C

      // Actuate Robot
      #line 1005 "potential_field.lit"
      // Update the EMA for the random force
      updatePositionEMA();
      // Calculate the total force
      Vector2 ft = getTotalForce();
      // Calculate the shortest angle between the force and the heading
      double theta = atan2(ft.y, ft.x) - pose[ID].heading;
      theta = atan2(sin(theta), cos(theta));
      // Calculate the velocities
      double omega = getAngularVelocity(theta);
      double v = getLinearVelocity(ft, theta);
      // Move the robot
      move(v, omega);
      #line 1035 "potential_field.lit"
      ++tickCounter;
      #line 1139 "potential_field.lit"
      if (ON_RANDOM_WALK && ID == 0
          && checkFrequency(GOAL_REGENERATION_FREQUENCY_HZ)) {
        generateGoal();
      }
      #line 1170 "potential_field.lit"
      //if (checkFrequency(2.0)) printDiagnostic();

#line 1299 "potential_field.lit"

      ros::spinOnce();
            // Need to call this function often to allow ROS to process
            // incoming messages
      rate.sleep();
            // Sleep for the rest of the cycle, to enforce the FSM loop rate
    }
  };


  void postInitialization() {

    // Post-Initialization
    #line 718 "potential_field.lit"
    // Get pose data
    ros::Time epoch(0.0);
    while (pose[ID].t == epoch && ros::ok())
      ros::spinOnce();
    // Reset EMA to pose
    resetPositionEMA();
    #line 817 "potential_field.lit"
    // Initialize random time generator
    srand(time(NULL));
    #line 1031 "potential_field.lit"
    tickCounter = 0;
    #line 1135 "potential_field.lit"
    if (ON_RANDOM_WALK && ID == 0) generateGoal();
    #line 1156 "potential_field.lit"
    ROS_INFO("Post-initialization completed.");

#line 1312 "potential_field.lit"

  }


  // Methods
  #line 75 "potential_field.lit"
  // Leader Attraction Method
  #line 122 "potential_field.lit"
  /*
    gamma:  attraction coefficient
    gx, gy: position of the goal
    rx, ry: position of the robot
   */
  Vector2 getLeaderAttraction(double gamma,
      double gx, double gy, double rx, double ry) {
    // Create the return struct
    Vector2 fa;
    // Get the distance to the goal
    double sx = gx - rx;
    double sy = gy - ry;
    // Calculate the vector conversion factor
    double ka = gamma * sqrt(sx*sx + sy*sy);
    // Calculate the vector of the force
    fa.x = ka * sx;
    fa.y = ka * sy;
    return fa;
  }
  #line 153 "potential_field.lit"
  Vector2 getLeaderAttraction() {
    Pose &robot = pose[ID];
    return getLeaderAttraction(ATTRACTION_CONSTANT_GAMMA,
        goalX, goalY, robot.x, robot.y);
  }

  #line 76 "potential_field.lit"
  #line 179 "potential_field.lit"
  // Follower Attraction Method
  #line 234 "potential_field.lit"
  /*
    alpha:    repulsion coefficient
    gamma:    attraction coefficient
    x0x, x0y: position of robot 0
    xjx, xjy: position of robot j
    r0, rj:   robot radii
    dSafe:    safe distance
    dFar:     "too far" distance
   */
  Vector2 getFollowerAttraction(double alpha, double gamma,
      double x0x, double x0y, double xjx, double xjy, double r0, double rj,
      double dSafe, double dFar) {
    // Create the return struct
    Vector2 fa;
    // Calculate the distance between the center of robot j to 0
    double sx = x0x - xjx;
    double sy = x0y - xjy;
    double sm = sqrt(sx*sx + sy*sy);
    // Subtract the radii of the robots
    double d = sm - (r0 + rj);
    // Calculate the vector conversion factor
    double ka;
    if (d < dSafe) {
      ka = -alpha / (d*d * sm);
    } else if (d < dFar) {
      fa.x = 0;
      fa.y = 0;
      return fa;
    } else {
      ka = gamma * d*d / sm;
    }
    // Calculate the vector of the force
    fa.x = ka * sx;
    fa.y = ka * sy;
    return fa;
  }
  #line 275 "potential_field.lit"
  Vector2 getFollowerAttraction() {
    Pose &x0 = pose[0];
    Pose &xj = pose[ID];
    return getFollowerAttraction(REPULSION_CONSTANT_ALPHA,
        ATTRACTION_CONSTANT_GAMMA, x0.x, x0.y, xj.x, xj.y, ROBOT_RADIUS_M,
        ROBOT_RADIUS_M, SAFE_DISTANCE_M, FAR_DISTANCE_M);
  }

  #line 180 "potential_field.lit"
  #line 302 "potential_field.lit"
  // Attraction Method
  #line 310 "potential_field.lit"
  Vector2 getAttraction() {
    if (ID == 0) {
      return getLeaderAttraction();
    } else {
      return getFollowerAttraction();
    }
  }

  #line 303 "potential_field.lit"
  #line 325 "potential_field.lit"
  // Repulsion Method
  #line 367 "potential_field.lit"
  /*
    alpha:    repulsion coefficient
    sim, siTheta: vector from end of laser (on wall) to sensor
    dSafe:    safe distance
    epsilon:  error constant
    beta:     relevance distance
   */
  Vector2 getRepulsion(double alpha, double sim, double siTheta, double dSafe,
      double epsilon, double beta) {
    // Create the return struct
    Vector2 fr;
    // Calculate the magnitude of the force
    double frm;
    if (sim < dSafe + epsilon) {
      frm = alpha / (epsilon*epsilon);
    } else if (sim < beta) {
      double di_ds = sim - dSafe;
      frm = alpha / (di_ds*di_ds);
    } else {
      fr.x = 0;
      fr.y = 0;
      return fr;
    }
    fr.x = frm * cos(siTheta);
    fr.y = frm * sin(siTheta);
    return fr;
  }
  #line 399 "potential_field.lit"
  /*
    i:  index of the laser range reading
   */
  Vector2 getRepulsion(size_t i) {
    // Get the magnitude
    float m = (*laserRanges)[i];
    // Calculate the angle
    double theta = laserAngleMin + i*laserAngleIncrement;
    // Calculate the vector
    return getRepulsion(REPULSION_CONSTANT_ALPHA, m, theta,
        SAFE_DISTANCE_M, ERROR_CONSTANT_EPSILON_M, RELEVANCE_DISTANCE_BETA_M);
  }

  #line 326 "potential_field.lit"
  #line 443 "potential_field.lit"
  // Other Repulsion Patch Method
  #line 460 "potential_field.lit"
  Vector2 getOtherRepulsion() {
    // Create the return vector
    Vector2 fp;
    // Add all the repulsive forces
    Pose &m = pose[ID];
    for (int i = 1; i < numRobots; ++i) {
      if (i == ID) continue;
      Pose &n = pose[i];
      double sx = m.x - n.x;
      double sy = m.y - n.y;
      fp += getRepulsion(REPULSION_CONSTANT_ALPHA, sqrt(sx*sx + sy*sy),
          atan2(sy, sx), SAFE_DISTANCE_M, ERROR_CONSTANT_EPSILON_M,
          RELEVANCE_DISTANCE_BETA_M);
    }
    // Apply the factor
    double factor = OTHER_REPULSION_FACTOR;
          // Got a weird linker error if this wasn't put in a local variable
    fp *= factor;
    return fp;
  }

  #line 444 "potential_field.lit"
  #line 561 "potential_field.lit"
  // Locate Leader Method
  #line 630 "potential_field.lit"
  /*
    leader:   the leader location struct to output to
    x0x, x0y: position of robot 0
    xjx, xjy: position of robot j
    r0:       leader radius
    epsilon:  error constant
   */
  void locateLeader(Leader &leader, double x0x, double x0y,
      double xjx, double xjy, double r0, double epsilon) {
    // Calculate the distance between the center of robot j to 0
    double sx = x0x - xjx;
    double sy = x0y - xjy;
    double sm = sqrt(sx*sx + sy*sy);
    double sTheta = atan2(sy, sx);
    // Calculate the threshold distance
    leader.distanceThreshold = sm - (r0 + epsilon);
    // Calculate the angle difference
    double dTheta = std::abs(atan2(r0, sm));
    // Calculate the min and max angles
    leader.thetaMin = sTheta - dTheta;
    leader.thetaMax = sTheta + dTheta;
  }
  #line 657 "potential_field.lit"
  void locateLeader(Leader &leader) {
    Pose &x0 = pose[0];
    Pose &xj = pose[ID];
    locateLeader(leader, x0.x, x0.y, xj.x, xj.y, ROBOT_RADIUS_M,
        ERROR_CONSTANT_EPSILON_M);
  }

  #line 562 "potential_field.lit"
  // Check for Leader Method
  #line 583 "potential_field.lit"
  /*
    leader:     the current leader location struct
    laserRange: laser range reading
    laserAngle: absolute angle of the reading
   */
  bool checkForLeader(const Leader &leader,
      float laserRange, double laserAngle) {
    return (laserAngle >= leader.thetaMin && laserAngle <= leader.thetaMax
        && laserRange >= leader.distanceThreshold);
  }
  #line 598 "potential_field.lit"
  bool checkForLeader(const Leader &leader, size_t i) {
    // Get the magnitude
    float m = (*laserRanges)[i];
    // Calculate the angle
    double theta = laserAngleMin + i*laserAngleIncrement;
    // Check for leader
    return checkForLeader(leader, m, theta);
  }

  #line 563 "potential_field.lit"
  #line 671 "potential_field.lit"
  // Position EMA Reset Method
  #line 706 "potential_field.lit"
  void resetPositionEMA() {
    // Reset to the robot's current position
    Pose &robot = pose[ID];
    positionEMA.x = robot.x;
    positionEMA.y = robot.y;
  }

  #line 672 "potential_field.lit"
  // Position EMA Update Method
  #line 729 "potential_field.lit"
  void updatePositionEMA() {
    // Discount the old value
    positionEMA *= 1 - EMA_COEFFICIENT_ETA;
    // Add the new value
    Pose &robot = pose[ID];
    positionEMA.x += EMA_COEFFICIENT_ETA * robot.x;
    positionEMA.y += EMA_COEFFICIENT_ETA * robot.y;
  }

  #line 673 "potential_field.lit"
  // Random Force Generator
  #line 762 "potential_field.lit"
  /*
    zeta: conversion constant
    p:    "staying put index"
   */
  Vector2 generateRandomForce(double zeta, double p) {
    // Create the return struct
    Vector2 f;
    // Generate the random angle
    double u = generateRandomAngle();
    // Calculate the force
    double k = zeta * p;
    f.x = k * cos(u);
    f.y = k * sin(u);
    return f;
  }
  #line 782 "potential_field.lit"
  Vector2 generateRandomForce() {
    // Calculate the "staying put index"
    Pose &robot = pose[ID];
    double ipx = robot.x - positionEMA.x;
    double ipy = robot.y - positionEMA.y;
    double p = 1 / sqrt(ipx*ipx + ipy*ipy);
    // Generate the force
    return generateRandomForce(CONVERSION_CONSTANT_ZETA, p);
  }

  #line 674 "potential_field.lit"
  // Random Angle Generator
  #line 809 "potential_field.lit"
  double generateRandomAngle() {
    return (double(rand()) / RAND_MAX)*2*M_PI - M_PI;
  }

  #line 675 "potential_field.lit"
  #line 833 "potential_field.lit"
  // Total Force Method
  #line 844 "potential_field.lit"
  Vector2 getTotalForce() {
    // Create the return struct, initialized with the attractive force.
    Vector2 ft = getAttraction();
    // Add the random force
    ft += generateRandomForce();
    // Get the number of laser data points
    size_t len = laserRanges->size();
    if (ID == 0) {
      // Leader does not need to filter for itself.
      for (size_t i = 0; i < len; ++i) {
        ft += getRepulsion(i);
      }
    } else {
      // Followers need to filter out the leader.
      Leader leader;
      locateLeader(leader);
      for (size_t i = 0; i < len; ++i) {
        if (!checkForLeader(leader, i)) {
          ft += getRepulsion(i);
        }
      }
    }
    // Patch because laser doesn't hit other robots
    ft += getOtherRepulsion();
    // Return the total force
    return ft;
  }

  #line 834 "potential_field.lit"
  #line 879 "potential_field.lit"
  // Angular Velocity Method
  #line 928 "potential_field.lit"
  /*
    kappa:    angular scaler
    theta:    angle between force and heading
    omegaMax: max allowable angular speed
   */
  double getAngularVelocity(double kappa, double theta, double omegaMax) {
    double raw = kappa * theta;
    return sgn(raw) * std::min(std::abs(raw), omegaMax);
  }
  #line 963 "potential_field.lit"
  double getAngularVelocity(double theta) {
    return getAngularVelocity(ANGULAR_SCALER_KAPPA, theta, ROTATE_SPEED_RADPS);
  }

  #line 880 "potential_field.lit"
  // Linear Velocity Method
  #line 940 "potential_field.lit"
  /*
    lambda:   linear scaler
    ftx, fty: total force
    theta:    angle between force and heading
    vMax:     max allowable linear speed
   */
  double getLinearVelocity(double lambda, double ftx, double fty,
      double theta, double vMax) {
    double raw = lambda * sqrt(ftx*ftx + fty*fty) * cos(theta);
    return (raw > 0) ? std::min(raw, vMax) : 0;
  }
  #line 969 "potential_field.lit"
  double getLinearVelocity(Vector2 &ft, double theta) {
    return getLinearVelocity(LINEAR_SCALER_LAMBDA, ft.x, ft.y, theta,
        FORWARD_SPEED_MPS);
  }

  #line 881 "potential_field.lit"
  // Sign Function
  #line 954 "potential_field.lit"
  double sgn(double a) {
    return (a > 0) - (a < 0);
  }

  #line 882 "potential_field.lit"
  #line 1039 "potential_field.lit"
  bool checkFrequency(float hz) {
    return tickCounter % int(30.0 / hz) == 0;
  }
  #line 1050 "potential_field.lit"
  // Is on Goal Method
  #line 1070 "potential_field.lit"
  bool isOnGoal(double gx, double gy, double ex, double ey, double sigma) {
    double sx = gx - ex;
    double sy = gy - ey;
    return sqrt(sx*sx + sy*sy) < sigma;
  }
  #line 1080 "potential_field.lit"
  bool isOnGoal() {
    return isOnGoal(goalX, goalY, positionEMA.x, positionEMA.y,
        GOAL_RADIUS_SIGMA_M);
  }

  #line 1051 "potential_field.lit"
  #line 1101 "potential_field.lit"
  // Goal Generator
  #line 1119 "potential_field.lit"
  void generateGoal() {
    // Generate random numbers
    double um = 0.5 + (double(rand()) / RAND_MAX);
    double ua = generateRandomAngle();
    // Calculate new goal
    Pose &robot = pose[ID];
    goalX = robot.x + um * cos(ua);
    goalY = robot.y + um * sin(ua);
    // Reset EMA to pose
    resetPositionEMA();
  }

  #line 1102 "potential_field.lit"
  #line 1160 "potential_field.lit"
  void printDiagnostic(const Vector2 &ft, double theta, double v, double omega) {
    ROS_INFO("%2d Pose:%6.2f,%6.2f,%+7.4f; F:%+10.4f,%+10.4f (%+7.4f);"
        " thetaD:%+7.4f; v:%10.4f; omega:%+9.4f; goal:%6.2f,%6.2f;"
        " pEMA:%6.2f,%6.2f; onGoal:%1d", ID, pose[ID].x, pose[ID].y,
        pose[ID].heading, ft.x, ft.y, atan2(ft.y, ft.x), theta, v, omega, goalX,
        goalY, positionEMA.x, positionEMA.y, isOnGoal());
  }

#line 1317 "potential_field.lit"


  // Parameters
  #line 14 "potential_field.lit"
  // Tunable motion controller parameters
  const static double FORWARD_SPEED_MPS = 2.0;
  const static double ROTATE_SPEED_RADPS = M_PI;
  #line 163 "potential_field.lit"
  // Parameters added for leader attraction
  const static double ATTRACTION_CONSTANT_GAMMA = 1.0;
  #line 289 "potential_field.lit"
  // Parameters added for follower attraction
  const static double REPULSION_CONSTANT_ALPHA = 1.0;
  const static double ROBOT_RADIUS_M = 0.3;
  const static double SAFE_DISTANCE_M = 0.75;
  const static double FAR_DISTANCE_M = 4.0;
  #line 426 "potential_field.lit"
  // Parameters added for repulsion
  const static double ERROR_CONSTANT_EPSILON_M = 0.03125;
  const static double RELEVANCE_DISTANCE_BETA_M = 8.0;
  #line 483 "potential_field.lit"
  // Other Repulsion
  const static double OTHER_REPULSION_FACTOR = 2.0;
  #line 797 "potential_field.lit"
  // Parameters added for random force
  const static double EMA_COEFFICIENT_ETA = 0.0625;
  const static double CONVERSION_CONSTANT_ZETA = 0.125;
  #line 984 "potential_field.lit"
  // Parameters added for force to velocity
  const static double ANGULAR_SCALER_KAPPA = 0.25;
  const static double LINEAR_SCALER_LAMBDA = 0.0625;
  #line 1089 "potential_field.lit"
  const static double GOAL_RADIUS_SIGMA_M = 1.25;
  #line 1146 "potential_field.lit"
  const static bool ON_RANDOM_WALK = false;
  const static double GOAL_REGENERATION_FREQUENCY_HZ = 0.2;

#line 1320 "potential_field.lit"


protected:
  ros::Publisher commandPub;
        // Publisher to the current robot's velocity command topic
  ros::Subscriber laserSub;
        // Subscriber to the current robot's laser scan topic
  std::vector<ros::Subscriber> poseSubs;
        // List of subscribers to all robots' pose topics
  std::vector<Pose> pose; // List of pose objects for all robots
  int ID;                 // 0-indexed robot ID
  int numRobots;          // Number of robots, positive value
  double goalX, goalY;    // Coordinates of goal

  // Member Variables
  #line 493 "potential_field.lit"
  // Laser Variables
  #line 417 "potential_field.lit"
  const std::vector<float> *laserRanges;
  double laserAngleMin;
  double laserAngleIncrement;
  #line 539 "potential_field.lit"
  sensor_msgs::LaserScan::ConstPtr laserMessage;

  #line 494 "potential_field.lit"
  #line 699 "potential_field.lit"
  Vector2 positionEMA;
  #line 1027 "potential_field.lit"
  unsigned tickCounter;

#line 1335 "potential_field.lit"

};


int main(int argc, char **argv) {
  std::cout << "Entered main." << std::endl;
  int robotID = -1, numRobots = 0;
  double goalX, goalY;
  bool printUsage = false;

  // Parse and validate input arguments
  if (argc <= 4) {
    printUsage = true;
  } else {
    try {
      robotID = boost::lexical_cast<int>(argv[1]);
      numRobots = boost::lexical_cast<int>(argv[2]);
      goalX = boost::lexical_cast<double>(argv[3]);
      goalY = boost::lexical_cast<double>(argv[4]);

      if (robotID < 0) { printUsage = true; }
      if (numRobots <= 0) { printUsage = true; }
    } catch (std::exception err) {
      printUsage = true;
    }
  }
  if (printUsage) {
    ROS_FATAL_STREAM("Usage: " << argv[0] <<
        " [ROBOT_NUM_ID] [NUM_ROBOTS] [GOAL_X] [GOAL_Y]");
    return EXIT_FAILURE;
  }

  ros::init(argc, argv, "potfieldbot_" + std::string(argv[1]));
        // Initiate ROS node
  ROS_INFO("ROS initialized.");
  if (numRobots == 1) {
    ros::NodeHandle n;
          // Create handle
    ROS_INFO("Node handle created: Single Mode");
    PotFieldBot robbie(n, robotID, numRobots, goalX, goalY);
          // Create new random walk object
    ROS_INFO("Node Initialized.");
    robbie.postInitialization();
    robbie.spin();  // Execute FSM loop
  } else {
    ros::NodeHandle n("robot_" + std::string(argv[1]));
          // Create named handle "robot_#"
    ROS_INFO("Node handle created: Group Mode");
    PotFieldBot robbie(n, robotID, numRobots, goalX, goalY);
          // Create new random walk object
    ROS_INFO("Node initialized.");
    robbie.postInitialization();
    robbie.spin();  // Execute FSM loop
  }

  return EXIT_SUCCESS;
};

