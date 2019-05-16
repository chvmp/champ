
#ifndef _QUADRUPEDLEG_H_
#define _QUADRUPEDLEG_H_
#include <QuadrupedLeg.h>

// QuadrupedLeg<3> lf_leg;
// RevoluteJoint lf_hip(0, 0.89, 0, 1.5708);
// RevoluteJoint lf_upper_leg(0.071, -0.76, 0.141, 0);
// RevoluteJoint lf_lower_leg(0, 1.49, 0.141, 0);



// RevoluteJoint lh_hip(0, 0.89, 0, 1.5708);
// RevoluteJoint lh_upper_leg(0.071, -0.76, 0.141, 0);
// RevoluteJoint lh_lower_leg(0, 1.49, 0.141, 0);

QuadrupedLeg<3> rf_leg;
// RevoluteJoint rf_hip(0, 0.73, 0, 1.5708);
// RevoluteJoint rf_upper_leg(-0.071, 0.89, 0.141, 0);
// RevoluteJoint rf_lower_leg(0, -1.53, 0.141, 0);

 RevoluteJoint rf_hip(0, 0.89, 0, 1.5708);
 RevoluteJoint rf_upper_leg(-0.071, -0.76, 0.141, 0);
 RevoluteJoint rf_lower_leg(0, 1.49, 0.141, 0);
// RevoluteJoint rh_hip(0, 0.89, 0, 1.5708);
// RevoluteJoint rh_upper_leg(-0.071, -0.76, 0.141, 0);
// RevoluteJoint rh_lower_leg(0, 1.49, 0.141, 0);



#endif
