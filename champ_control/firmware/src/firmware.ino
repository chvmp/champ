#include <ros.h>
#include <ros/time.h>
#include <champ_msgs/Point.h>
#include <Geometry.h>
#include <Geometry.h>
// #include <QuadrupedLeg.h>

#include <DH.h>

#define CONTROL_RATE 100

ros::NodeHandle nh;
champ_msgs::Point point_msg;
ros::Publisher point_pub("point_debug", &point_msg);


void setup()
{
  rf_leg.addLink(rf_hip);
  rf_leg.addLink(rf_upper_leg);
  rf_leg.addLink(rf_lower_leg);

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(point_pub);

  while (!nh.connected())
  {
      nh.spinOnce();
  }
  nh.loginfo("CHAMP CONNECTED");
  delay(1);
}

void loop() { 
    static unsigned long prev_ik_time = 0;

    //this block drives the robot based on defined rate
    if ((millis() - prev_ik_time) >= (1000 / CONTROL_RATE))
    {
      Transformation target;
       
      target.p = rf_leg.getFootPose().p;

      double joints[3];

      rf_leg.getJointAngles(target, joints);

      char buffer[50];
      sprintf (buffer, "Joint0  : %f", joints[0]);
      nh.loginfo(buffer);

      sprintf (buffer, "Joint1  : %f",joints[1]);
      nh.loginfo(buffer);

      sprintf (buffer, "Joint2  : %f", joints[2]);
      nh.loginfo(buffer);

      point_msg.x = target.X();
      point_msg.y = target.Y(); 
      point_msg.z = target.Z();
      point_pub.publish(&point_msg);
      prev_ik_time = millis();
    }
    nh.spinOnce();
}
