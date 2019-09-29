#include <BasicLinearAlgebra.h>

/*
 * This example sketch shows how the template parameter can be nested to form high dimensional Matrices called 'Tensors'.
 * I should point out that I'm not really advocating using arduinos for Tensor computation, this is just an interesting
 * way of showing the flexbility and extensibility of the Matrix library
 */

// All the functions in BasicLinearAlgebra are wrapped up inside the namespace BLA, so specify that we're using it like so:
using namespace BLA;

void setup() 
{
  Serial.begin(115200);

  // In addition to the Matrix dimensions, the Matrix template has a third parameter which when not specified just defaults to the type: Array<rows,cols,float>.
  // This third parameter specifies how the Matrix should store it's elements such that the way that the Matrix serves up it's elements ca be customised
  // (see the CustomMatrix example for more). The Array type simply means that the Matrix stores it's elements in a big array of size rows x cols.

  // In any case, written in full, a Matrix declaration looks like this:
  BLA::Matrix<3,3,Array<3,3,float> > floatA;

  // The default underlying type of the Array class's array is float. If you want to use a different type, say int for example, then just pass it as a template parameter like so:
  BLA::Matrix<3,3,Array<3,3,int> > intA;

  // I find this to be a bit cumbersome though so I've defined a template alias called ArrayMatrix which can be used like so:
  ArrayMatrix<3,3,int> intB; // intA and intB are identical at this point

  // From here you'll be able to do everything you'd be able to do with a float Matrix, but with int precision and memory useage.
  int array[3][3] = {{1,2,3},{4,5,6},{7,8,9}};
  intA = array;

  // You can actually pass any datatype you like to the template and it'll do it's best to make a Matrix out of it.
  ArrayMatrix<3,3,unsigned char> charA;
  ArrayMatrix<3,3,double> doubleA; // etc

  // This includes parameters of type Matrix, meaning that you can declare matrices of more than two dimensions. For example:
  ArrayMatrix<4,4,ArrayMatrix<4> > cubeA, cubeB; // a 4x4x4 Matrix (3rd order tensor)

  // And so on:
  ArrayMatrix<2,2,ArrayMatrix<2,2> > hyperA; // a 2x2x2x2 dimensional Matrix (4th order tensor)
  ArrayMatrix<3,3,ArrayMatrix<3,3, Matrix<3,3> > > tensorA; // a 3x3x3x3x3x3 dimensional Matrix (6th order tensor)

  // You can access the elements of an arbitrary rank tensor with the brackets operator like so:
  cubeA(0,1)(1) = cubeB(2,3)(3) = 56.34;
  hyperA(1,0)(1,1) = 56.34;
  tensorA(2,0)(1,2)(1,1) = 0.056;

  // Addition, subtraction and transposition all work as usual
  cubeA + cubeB;

  // As does concatenation
  ArrayMatrix<4,8,ArrayMatrix<4> > cubeAleftOfcubeB = cubeA || cubeB;

  // You can also do multiplication on square tensors with an even rank
  for(int i = 0; i < 2; i++)
      for(int j = 0; j < 2; j++)
          for(int k = 0; k < 2; k++)
              for(int l = 0; l < 2; l++)
                  hyperA(i,j)(k,l) = i+j+k+l;

  ArrayMatrix<2,2,ArrayMatrix<2,2> > hyperB = (hyperA * hyperA);

  // Everything can be printed too
  Serial << "Hyper B: " << hyperB;

  // Inversion doesn't work. If it did, it'd probably take quite a while for arduino to calculate anyway so maybe it's for the best
}

void loop() { }
