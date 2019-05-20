#include <quadruped_base.h>

#define BASE_TO_HIP_X 0.175
#define BASE_TO_HIP_Y 0.105

RevoluteJoint lf_hip(0, 0, 0, 1.5708);
RevoluteJoint lf_upper_leg(0.06, 0, 0.141, 0);
RevoluteJoint lf_lower_leg(0, 0, 0.141, 0);
QuadrupedLeg lf_leg(lf_hip, lf_upper_leg, lf_lower_leg, BASE_TO_HIP_X, BASE_TO_HIP_Y, 0, 0, PI/2, PI/2);

RevoluteJoint rf_hip(0, 0, 0, 1.5708);
RevoluteJoint rf_upper_leg(-0.06, 0, 0.141, 0);
RevoluteJoint rf_lower_leg(0, 0, 0.141, 0);
QuadrupedLeg rf_leg(rf_hip, rf_upper_leg, rf_lower_leg, BASE_TO_HIP_X, -BASE_TO_HIP_Y, 0, 0, PI/2, PI/2);;

RevoluteJoint lh_hip(0, 0, 0, 1.5708);
RevoluteJoint lh_upper_leg(0.06, 0, 0.141, 0);
RevoluteJoint lh_lower_leg(0, 0, 0.141, 0);
QuadrupedLeg lh_leg(lh_hip, lh_upper_leg, lh_lower_leg, -BASE_TO_HIP_X, BASE_TO_HIP_Y, 0, 0, PI/2, PI/2);

RevoluteJoint rh_hip(0, 0, 0, 1.5708);
RevoluteJoint rh_upper_leg(-0.06, 0, 0.141, 0);
RevoluteJoint rh_lower_leg(0, 0, 0.141, 0);
QuadrupedLeg rh_leg(rh_hip, rh_upper_leg, rh_lower_leg, -BASE_TO_HIP_X, -BASE_TO_HIP_Y, 0, 0, PI/2, PI/2);
