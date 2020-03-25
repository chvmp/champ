#ifndef QUADRUPED_DESCRIPTION_H
#define QUADRUPED_DESCRIPTION_H

#include <quadruped_base/quadruped_base.h>

champ::Joint lf_hip(0.175, 0.105, 0, 0, 0, 0);
champ::Joint lf_upper_leg(0, 0.06, 0, 0, 0, 0);
champ::Joint lf_lower_leg(0, 0, -0.141, 0, 0, 0);
champ::Joint lf_foot(0, 0, -0.141, 0, 0, 0);
champ::QuadrupedLeg lf_leg(lf_hip, lf_upper_leg, lf_lower_leg, lf_foot);

champ::Joint rf_hip(0.175, -0.105, 0, 0, 0, 0);
champ::Joint rf_upper_leg(0, -0.06, 0, 0, 0, 0);
champ::Joint rf_lower_leg(0, 0, -0.141, 0, 0, 0);
champ::Joint rf_foot(0, 0, -0.141, 0, 0, 0);
champ::QuadrupedLeg rf_leg(rf_hip, rf_upper_leg, rf_lower_leg, rf_foot);;

champ::Joint lh_hip(-0.175, 0.105, 0, 0, 0, 0);
champ::Joint lh_upper_leg(0, 0.06, 0, 0, 0, 0);
champ::Joint lh_lower_leg(0, 0, -0.141, 0, 0, 0);
champ::Joint lh_foot(0, 0, -0.141, 0, 0, 0);
champ::QuadrupedLeg lh_leg(lh_hip, lh_upper_leg, lh_lower_leg, lh_foot);

champ::Joint rh_hip(-0.175, -0.105, 0, 0, 0, 0);
champ::Joint rh_upper_leg(0, -0.06, 0, 0, 0, 0);
champ::Joint rh_lower_leg(0, 0, -0.141, 0, 0, 0);
champ::Joint rh_foot(0, 0, -0.141, 0, 0, 0);
champ::QuadrupedLeg rh_leg(rh_hip, rh_upper_leg, rh_lower_leg, rh_foot);

#endif