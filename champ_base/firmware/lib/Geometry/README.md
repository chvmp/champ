# Geometry
A library for working with points, rotations and coordinate transformations in 3D space.

**This library depends on BasicLinearAlgebra (available in the library manager) so be sure to install that too**

Geometry defines three classes to represent 3D coordinates, rotation matrices and transformation matrices. All three classes inherit from the Matrix class so they're all able to do basic matrix arithmetic. Most importantly, multiplication between Points and Rotations or Rotations and Rotations allows those objects to be rotated in 3D. There's also a few useful methods for converting between rotation matrices and Euler angles as well as finding the cross product of two vectors and so on. 

These kinds of calculations are really helpful for projects involving 3D motion like robot arm kinematics or working with gyro sensor data. Enjoy!

To get started, just go through the HowToUse sketch in the examples folder; that'll explain everything that most users will need to know to work with the library. There's also the KinematicChain example which uses the library to calculate the forward and inverse kinematics for an arbitrary length kinematic chain. The inverse kinematics are found using gradient descent with a jacobian transpose so there's room for improvement. Feel free to subclass KinematicChain and override that method with something better!

