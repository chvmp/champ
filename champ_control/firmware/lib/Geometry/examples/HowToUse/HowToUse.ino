#include <Geometry.h>

/*
 * This example sketch should show you everything you need to know in order to work with the Geometry library. It's a faily long explantion
 * but try to read through it carefully. It assumes a bit of knowledge about rotation and transformation matrices so if you're not familiar
 * with them, then have a read through wikipedia.org/wiki/Rotation_matrix . Transformation matrices are used pretty heavily in
 * manipulators so for more information on them, have a read of wikipedia.org/wiki/Denavit-Hartenberg_parameters
 */

void setup()
{
  Serial.begin(115200);

  // This library defines three classes, Point, Rotation and Transformation

  // Point represents a coordinate in 3D space
  Point p1, p2;

  // You can access it's elements using the () operator
  p1(0) = -6.45;
  p1(2) = 5.34;
  p2(1) = 7.67;

  // Or with the methods X(), Y() & Z()
  p1.Y() = 4.27;
  p2.X() = -9.12;
  p2.Z() = 3.32;

  // Point, as well as the other two classes inherit from the Matrix class so you can do all the things with them as you would a matrix of the same size

  // You can do arithmetic with them
  Point d = p1 - p2;

  // And print them to Serial
  Serial << "p2 relative to p1 = " << d << '\n';

  // And more, but read the BasicLinearAlgebra library's HowToUse example for more details

  // Point also has a few extra methods for calculating the Magnitude, as well as the DotProduct and CrossProduct
  Serial << "distance between p1 & p2 = " << d.Magnitude() << "\n\n";

  // Rotation represents a rotated coordinate frame (a set of x/y/z axes) expressed with respect to a base frame.
  Rotation R1, R2;

  // You can rotate a these matrices about the X, Y or Z axis using the Rotate methods. These operations are the equivilent of premultiplying the
  // rotation matrix by the corresponding matrix described in the Basic rotations heading of the Rotation Matrix wikipedia page.
  // Note that all angles should be specified in radians.
  R1.RotateZ(M_PI_4);
  R1.RotateY(0.9);
  R1.RotateX(0.2);

  // And you can print them just as you would a matrix
  Serial << "R1 = " << R1 << '\n';

  // You can think of a rotation matrix as just three 3x1 vectors which describe the direction of the x/y/z axes of a rotated coordinate frame with respect to a base frame.
  // To illustrate that let's take the columns of R1 and set them to three Point instances
  Point R1x, R1y, R1z;
  R1x = R1.Submatrix(Slice<0,3>(),Slice<0,1>());
  R1y = R1.Submatrix(Slice<0,3>(),Slice<1,2>());
  R1z = R1.Submatrix(Slice<0,3>(),Slice<2,3>());

  // Now looking at the magnitudes we can see that they're all unit vectors
  Serial << "R1 X,Y & Z magnitudes: " << R1x.Magnitude() << ", " << R1y.Magnitude() << " & " << R1z.Magnitude() << "\n";

  // And we if we take the dot product of each of three combinatations of axes we can see that they're all orthogonal to each other
  Serial << "R1x . R1y = " << R1x.DotProduct(R1y) << ", R1y . R1z = " << R1y.DotProduct(R1z) << " & R1z . R1x = " << R1z.DotProduct(R1x) << "\n\n";

  // Point being, it's easy to understand rotation matrices if you think of their columns as a set of unit vectors describing the rotated x,y & z axes.
  // Also, for a rotation matrix to behave like it should, it's columns need to be orthogonal and have a magnitude of 1. For that reason it's better to work with
  // them via the rotate methods, but if you really want to you can access the individual elements using the () operator like so:
  R1(2,2) = R1z(2);

  // You can also populate a rotation matrix using euler angles (roll, pitch, yaw) which is a good way to work with gyro sensor data without having to worry about gimbal lock
  R2.FromEulerAngles(0.2, 0.9, M_PI_4);

  // This is analogous to making an X, followed by a Y and then an Z axis rotation, like so:
  Rotation R3;
  R3.RotateX(0.2);
  R3.RotateY(0.9);
  R3.RotateZ(M_PI_4); // pi / 4

  // Note that the Rotate methods are incremental (ie they add to the rotation already represented by the rotation matrix) whereas FromEulerAngles just populates them.
  // So these operations are only analogous if the R3 matrix example isn't already rotated.

  // Also note that when dealing with euler angles, the order of the rotations is important. Even though R1 and R2 are composed of the same rotations about the same axes,
  // the rotation are applied in a different order which results in a different final rotation
  Serial << "R1 is not the same as R2:\nR1 = " << R1 << "\nR2 = " << R2 << "\n\n";

  // The order of rotations euler angles goes: X, Y, Z; meaning that R2 and R3 are equivilent:
  Serial << "But R2 is the same as R3:\nR2 = " << R2 << "\nR3 = " << R3 << "\n\n";

  // You can also convert a Rotation back into euler angles as follows. Note that a one rotation matrix can be described by two different sets of euler angles,
  // this function returns both as the rows of a 3x2 matrix
  Serial << "R3 as euler angles: " << R3.ToEulerAngles()  << "\n\n";

  // You can rotate Points by premultiplying them with a Rotation like so:
  Point p3 = R2 * p1;
  Point p4 = R2 * p2;

  // The points will be rotated to different coordinates
  Serial << "p1 = " << p1 << "\np3 = " << p3 << "\n";

  // But obviously the distance between them stays the same
  Serial << "distance between p3 & p4 = " << (d = p3 - p4).Magnitude() << "\n\n";

  // Likewise you can rotate rotations by multiplying them together. Formally speaking, to rotate R1 by a rotation represented by R2 you must premultiply R1 by R2 like so:
  Rotation R4 = R2 * R1;

  // Rotation matrices have the property that their transpose is their inverse so we can undo a rotation by premultiplying by the transpose. So, the following statement rotates R1, then rotates it back again:
  Serial << "R1 =             " << R1 << "\nR2^T * R2 * R1 = " << (~R2 * R2 * R1) << "\n\n";

  // Last is the Transformation class which is the aggregation of a Point and a Rotation. A transformation matrix represents a set of rotated x, y, z axes whose origin is at some coordinate.
  // is a coordinate frame whose origin is located at a particular point. They're useful for representing a position plus an orientation
  Transformation T1, T2;

  // You can access the origin (Point) of the transformtion directly through it's member p and you can access the rotation of the frame through R.
  // These are just instances of Point and Rotation respectively so everything we've covered so far still applies to them:
  T1.p.X() = 23.54;
  T1.R.RotateZ(M_PI_4); // pi / 2

  // Otherwise, you can treat the Transformation class as a 4x4 matrix arranged like this {{R p},{0 0 0 1}}. You can mutliply them together to combine two transformations like so:
  Transformation T3 = T1 * T2;

  // Transformation also let's you rotate it's orientation as well as it's origin simultaneously through it's own Rotate methods:
  T3.RotateZ(0.343);
  T3.RotateX(1.234);

  // You can also translate the origin of the transformation matrix like so:
  T3.Translate(22.1, 56.43, 100.54);

  // And of course you can print them to Serial in the usual way, note that they'll print out just like a 4x4 matrix
  Serial << "T3 = " <<  T3 << "\n";

  // For more on Transformation matrices, have a look at the KinematicChain example
}

void loop() { }

