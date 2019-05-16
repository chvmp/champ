#include <ros.h>
#include <ros/time.h>
#include <champ_msgs/Point.h>
#include <Geometry.h>

/* This example shows ho you can use the Geometry library to calculate the forward and inverse kinematics of an arbitrary length kinematic chain. The kinematic chain is specified
* Denavit-Hartenburg parameters, if you're not familiar with them watch this tutorial https://www.youtube.com/watch?v=rA9tm0gTln8 . The forward kinematics calculates the location of the
* end effector given a set of joint angles defined by D-H parameters stored in the 'chain' member. The inverse kinematics (IK) calculate a set of joint angles given a position for the end effector.
* In doing so it also sets these angles on the links in the chain member.
*/

// Link stores the D-H parameters for one link in the chain. It's an abstract base class so to use it you have to subclass it and define the Move function, more on that later though
class Link
{
public:
 float d, theta, r, alpha;

 Link(float _d, float _theta, float _r, float _alpha) : d(_d), theta(_theta), r(_r), alpha(_alpha) { }
 virtual void Move(float amount) = 0;
};

// KinematicChain manages the links and implements the forward and inverse kinematics
template<int maxLinks> class KinematicChain
{
   // A few variables used in the inverse kinematics defined here to save re-allocating them every time inverse kinematics is called
   Point deltaPose;
   Matrix<3,1> jacobian;
   Matrix<maxLinks> deltaAngles;
   Transformation currentPose, perturbedPose;

   // The number of links addedto the chain via AddLink
   unsigned int noOfLinks;

 public:
   // An array containing all the D-H parameters for the chain
   Link *chain[maxLinks];

   KinematicChain() { noOfLinks = 0; }

   // Add a link - it's D-H parameters as well as a function pointer describing it's joint movement
   void AddLink(Link &l)//float d, float theta, float r, float alpha, void (*move)(Link&, float))
   {
     if(noOfLinks == maxLinks)
       return;

     chain[noOfLinks++] = &l;
   }

   int NoOfLinks() { return noOfLinks; }

   // Transforms pose from the end effector coordinate frame to the base coordinate frame.
   Transformation &ForwardKinematics(Transformation &pose)
   {
     for(int i = noOfLinks - 1; i >= 0; i--)
     {
       // These four operations will convert between two coordinate frames defined by D-H parameters, it's pretty standard stuff
       pose.RotateX(chain[i]->alpha);
       pose.Translate(chain[i]->r, 0,0);
       pose.Translate(0,0,chain[i]->d);
       pose.RotateZ(chain[i]->theta);
     }
    pose.RotateX(-1.5708);

    return pose;
   }

   // Handy overload to save having to feed in a fresh Transformation every time
   Transformation ForwardKinematics()
   {
     currentPose = Identity<4,4>();
     return ForwardKinematics(currentPose);
   }

   // Calculates the joints angles Transforms targetPose to the R^chainsize domain of link angles and sets it on the internal chain array
   virtual Transformation &InverseKinematics(Transformation &targetPose, int maxIterations = 1000, float convergenceSpeed = 0.00001, float closeEnough = 0.1, float perturbance = 0.001)
   {
     // Inverse kinematics doesn't have a general solution so we have to solve it iteratively
     for (int it = 0; it < maxIterations; it++)
     {
       // First find the current end effector position
       currentPose = Identity<4,4>();
       ForwardKinematics(currentPose);

       // And find the error between the target and the current pose
       deltaPose = currentPose.p - targetPose.p;

       // If the search gets close enough then just break
       if (deltaPose.Magnitude() < closeEnough)
         break;

       // Now we move each joint a little bit and see how it affects the position of the end effector.
       for (unsigned int link = 0; link < noOfLinks; link++)
       {
         // Move the link a little bit
         perturbedPose = Identity<4,4>();
         chain[link]->Move(perturbance);

         // Find the change in end effector position
         ForwardKinematics(perturbedPose);

         // And set the link back to where it was for now
         chain[link]->Move(-perturbance);

         // Now divide the change in x/y/z position by the amount by which we moved the joint to find a jacobian in the form of {dx/dtheta, dy/dtheta, dz/dtheta}
         jacobian = (currentPose.p - perturbedPose.p) * (1 / perturbance);

         // Now calculate a change in joint angle to bring the chain towards the target position. The joint angle / position relatioship is really non-linear so the
         // jacobian won't be valid very far from the operating point and it's better to move only a little bit at a time; this is handled with the convergeSpeed parameter
         // Ideally we'd find the pseudoinverse of the jacobian here, but that's quite a bit of doing. For this example we'll just use the transpose as an inverse of sorts.
         deltaAngles(link) = (~jacobian * deltaPose)(0) * convergenceSpeed;
       }

       // Update the link angles
       for (unsigned int link = 0; link < noOfLinks; link++)
         chain[link]->Move(deltaAngles(link));
     }

     return currentPose;
   }
};

// In addition to the D-H parameters, the joint also needs to specify how the D-H parameters change in response to it's movement. This let's the inverse kinematics algorithm know how useful the joints movement is in reaching
// the goal position. You can define how the joint moves by subclassing Link and overriding the Move function. The function should modify the link's D-H parameters in some way proportional to the 'amount' parameter.

// For example, we can define a revolute joint which changes the theta D-H parameter when it moves
class RevoluteJoint : public Link
{
public:
 RevoluteJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
 void Move(float amount) { theta += amount; }
};

// We can define a prismatic joint which changes the r parameter. We migh also throw in a parameter 'stiffness' to make the joint more reluctant move in the IK
class PrismaticJoint : public Link
{
 float stiffness;
public:
 PrismaticJoint(float d, float theta, float r, float alpha, float _stiffness = 1) : Link(d, theta, r, alpha), stiffness(_stiffness) { }
 void Move(float amount) { r += (amount * stiffness); }
};

// We could even define a joint that doesn't move at all. The IK will effectively ignore this one
class ImmobileJoint : public Link
{
public:
 ImmobileJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
 void Move(float amount) { }
};
// =======================================START HERE==================================
// You get the idea. One thing we can't do is specify joints with more than 1DOF. For ball-and-socket joints and so on, just define multiple 1DOF joints on top of each other (D-H parameters all zero).
 
 #define CONTROL_RATE 100
 KinematicChain<3> k;
 Transformation target;

 // Next let's configure some links to give to the kinematic chain
 //link(d, theta, r, alpha)
 //working from hip link
 RevoluteJoint l1(0, 0.89, 0, 1.5708);
 RevoluteJoint l2(-0.071, -0.76, 0.141, 0);
 RevoluteJoint l3(0, 1.49, 0.141, 0);

ros::NodeHandle nh;
champ_msgs::Point point_msg;
ros::Publisher point_pub("point_debug", &point_msg);
void get_joint_angles(KinematicChain<3> &leg, Transformation &target, double *joints);

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(point_pub);

  while (!nh.connected())
  {
      nh.spinOnce();
  }
  nh.loginfo("CHAMP CONNECTED");
  delay(1);
 // We'll start by declaring a chain with up to 10 links, we'll only be adding 6 but that doesn't matter

 // and now add them to the chain
 k.AddLink(l1);
 k.AddLink(l2);
 k.AddLink(l3);
}

void loop() { 
    static unsigned long prev_ik_time = 0;

    //this block drives the robot based on defined rate
    if ((millis() - prev_ik_time) >= (1000 / CONTROL_RATE))
    {
      Transformation lf_tf;
       
      lf_tf.p = k.ForwardKinematics().p;

      double joints[3];

      get_joint_angles(k, lf_tf, joints);

      char buffer[50];
      sprintf (buffer, "Joint0  : %f", joints[0]);
      nh.loginfo(buffer);

      sprintf (buffer, "Joint1  : %f",joints[1]);
      nh.loginfo(buffer);

      sprintf (buffer, "Joint2  : %f", joints[2]);
      nh.loginfo(buffer);

      point_msg.x = lf_tf.X();
      point_msg.y = lf_tf.Y(); 
      point_msg.z = lf_tf.Z();
      point_pub.publish(&point_msg);
      prev_ik_time = millis();
    }
    nh.spinOnce();
}
void get_joint_angles(KinematicChain<3> &leg, Transformation &target, double *joints){
  joints[0] = -(atan(target.p.Z() / target.p.X()) - ( 1.5708 - acos(k.chain[1]->d / sqrt(pow(target.p.Z(),2) + pow(target.p.X(), 2)))));
  target.RotateY(joints[0]);
  // ik for knee forward
  // joints[2] = acos( (pow(target.p.X(),2) + pow(target.Y(),2) - pow(leg.chain[1]->r ,2) - pow(leg.chain[2]->r ,2)) / (2 * leg.chain[1]->r * leg.chain[2]->r) );
  // joints[1] = atan(target.p.Y() / target.p.X()) - atan( (leg.chain[2]->r * sin(joints[2])) / (leg.chain[1]->r + (leg.chain[2]->r * cos(joints[2]))));
  
  // reverse
  joints[2] = -acos((pow(target.p.X(),2) + pow(target.Y(),2) - pow(leg.chain[1]->r ,2) - pow(leg.chain[2]->r ,2)) / (2 * leg.chain[1]->r * leg.chain[2]->r));
  joints[1] = (atan(target.p.Y() / target.p.X()) - atan( (leg.chain[2]->r * sin(joints[2])) / (leg.chain[1]->r + (leg.chain[2]->r * cos(joints[2])))));
  target.RotateY(-joints[0]);

} 