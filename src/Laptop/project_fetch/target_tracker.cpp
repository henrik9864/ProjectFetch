#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
//#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <vector>
#include <QTransform>
#include <QColor>
#include <std_msgs/Float32MultiArray.h>
#include <std_srvs/Empty.h>

//Kjore framover: linear.x > 0

int GetSign(float num){
  return num/fabs(num);
}

void timerCallBack(const ros::TimerEvent&);

struct Navigation {
  float turnAngles;
  float deadZone = 2;
  bool aligned = false;
  bool clawReady = false;
  bool grabbed = false;
  bool objectInView = false;
  bool timerPrimed = false;

  ros::NodeHandle nh;
  ros::Publisher pubControl;
  ros::Timer timer;

  Navigation(ros::NodeHandle nh) {
    this->nh = nh;
    turnAngles = 0;
    pubControl = nh.advertise<geometry_msgs::Twist>("car_ctrl", 10);
  }

  void control() {
    if (grabbed)
      return;

    geometry_msgs::Twist output;
    if (turnAngles != 0) {      
      int dir = GetSign(turnAngles);

      geometry_msgs::Vector3 angular;
      angular.z = turnAngles;
      output.angular = angular;

      if (GetSign(turnAngles - dir) != GetSign(turnAngles)) {
        turnAngles = 0;
      }
        
      else
        turnAngles -= dir;
    }
    else {
      geometry_msgs::Vector3 linear;
      linear.x = 1;
      output.linear = linear;
    }
    
    pubControl.publish(output);
  }

  void SetTurnAngle(float angle){
    if (turnAngles != 0)
      return;

    if (angle * 30 < deadZone) {
      aligned = true;
      if (!clawReady) {
        ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("ready_claw");
        std_srvs::Empty srv;
        client.call(srv);
        clawReady = true;
      }
      return;
    }

    aligned = false;
    turnAngles = angle * 30;
  }

  void primeTimer() {
    timer = nh.createTimer(ros::Duration(1.5), timerCallBack, true);
    timerPrimed = true;
  }
};

Navigation *navigator;

void timerCallBack(const ros::TimerEvent& event) {
  navigator->grabbed = true;
}

void object_callback(const std_msgs::Float32MultiArrayConstPtr &msg)
{
  const std::vector<float> &data = msg->data;;
  if (!data.size()) {
    if (!navigator->objectInView && navigator->aligned && navigator->clawReady && !navigator->timerPrimed) {
      navigator->primeTimer();
    }
    navigator->objectInView = false;
    return;
  }
    

  int id = data[0];
  float width = data[1];
  float height = data[2];

  QTransform qth(data[3], data[4], data[5], data[6], data[7], data[8], data[9], data[10], data[11]);
  QPointF topLeft = qth.map(QPointF(0, 0));
  QPointF topRight = qth.map(QPointF(width, 0));
  QPointF bottomLeft = qth.map(QPointF(0, height));
  QPointF bottomRight = qth.map(QPointF(width, height));

  QPointF center = (topLeft + bottomRight) / 2;
  float percent_x = (center.x() - 640) / 640;

  //printf("Heloo world")
  navigator->SetTurnAngle(percent_x);
  navigator->objectInView = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "target_tracker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  ros::Subscriber obj_sub = nh.subscribe("/stereo/objects", 1, object_callback);

  navigator = new Navigation(nh);
  while(ros::ok()) {
    navigator->control();
    ros::spinOnce();
  }
  return 0;
}
