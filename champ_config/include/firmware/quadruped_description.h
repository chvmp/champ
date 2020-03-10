#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

Joint lf_hip(0.175, 0.105, 0, 0, 0, 0);
Joint lf_upper_leg(0, 0.06, 0, 0, 0, 0);
Joint lf_lower_leg(0, 0, -0.141, 0, 0, 0);
Joint lf_foot(0, 0, -0.141, 0, 0, 0);
QuadrupedLeg lf_leg(lf_hip, lf_upper_leg, lf_lower_leg, lf_foot);

Joint rf_hip(0.175, -0.105, 0, 0, 0, 0);
Joint rf_upper_leg(0, -0.06, 0, 0, 0, 0);
Joint rf_lower_leg(0, 0, -0.141, 0, 0, 0);
Joint rf_foot(0, 0, -0.141, 0, 0, 0);
QuadrupedLeg rf_leg(rf_hip, rf_upper_leg, rf_lower_leg, rf_foot);;

Joint lh_hip(-0.175, 0.105, 0, 0, 0, 0);
Joint lh_upper_leg(0, 0.06, 0, 0, 0, 0);
Joint lh_lower_leg(0, 0, -0.141, 0, 0, 0);
Joint lh_foot(0, 0, -0.141, 0, 0, 0);
QuadrupedLeg lh_leg(lh_hip, lh_upper_leg, lh_lower_leg, lh_foot);

Joint rh_hip(-0.175, -0.105, 0, 0, 0, 0);
Joint rh_upper_leg(0, -0.06, 0, 0, 0, 0);
Joint rh_lower_leg(0, 0, -0.141, 0, 0, 0);
Joint rh_foot(0, 0, -0.141, 0, 0, 0);
QuadrupedLeg rh_leg(rh_hip, rh_upper_leg, rh_lower_leg, rh_foot);

#endif