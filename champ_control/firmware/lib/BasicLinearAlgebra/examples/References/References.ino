#include <BasicLinearAlgebra.h>

/*
 * This example sketch shows how to use a reference matrix to isolate a section of a larger matrix and use it in arithmetic with smaller matrices of the
 * appropriate dimensions.
 */

// All the functions in BasicLinearAlgebra are wrapped up inside the namespace BLA, so specify that we're using it like so:
using namespace BLA;

void setup()
{
  Serial.begin(115200);

  // If you've been through the HowToUse example you'll know that you can allocate a Matrix like so:
  BLA::Matrix<8,8> bigMatrix;

  // And as before it's a good idea to fill the matrix before we use it
  bigMatrix.Fill(0);

  // Now we'll make a refence matrix which refers to bigMatrix. The reference has no internal memory of it's own, it just goes and gets whatever it needs from bigMatrix when it takes part in
  // an operation. It'll also stores the result of an operation there if needs be too. Reference matrices are handy in that we can use them to select just a submatrix of the original
  // and use that in operations with smaller matrices.

  // To allocate a reference matrix you can use the RefMatrix type. RefMatrix is a template class just like Matrix but it has one extra template parameter that needs filling in, that is, the type
  // of matrix that it refers to. In this example we'll just be referring to regular Matrices so that parameter should look like Array<N,M> where N and M are the number of rows and columns of the
  // matrix being referred to (the parent) respectively.

  // When declaring a RefMatrix, it needs to be told which matrix it's referring to and what the offset is between it's (0,0)
  // and that of it's parent's (0,0) element. To do that you can use the Submatrix method of the Matrix class. The Submatrix method takes two Ranges for selecting a set of rows and columns respectively.
  // The template parameter (the one in the <>) indicates how many elements to select while the other indicates the offset between the parent matrix and the reference.

  // So for example, to create a 4x4 reference to bigMatrix starting at element (4,2) the declaration would be as follows:
  RefMatrix<4,4,Array<8,8>> bigMatrixRef(bigMatrix.Submatrix(Slice<4,8>(),Slice<2,6>()));

  // If we set the (0,0) element of bigMatrixRef, we're effectively setting the (4,2) element of bigMatrix. So let's do that
  bigMatrixRef(0,0) = 45.67434;

  // And we can see that the original matrix has been set accordingly
  Serial << "bigMatrix(4,2): " << bigMatrix(4,2) << "\n";

  // The submatrix function actually returns a RefMatrix so if you like you can just use it directly. For example you can set a section of bigMatrix using an array like so:
  float arr[4][4] = {{23.44,43.23,12.45,6.23},{93.94,27.23,1.44,101.23},{1.23,3.21,4.56,8.76},{12.34,34.56,76.54,21.09}};
  bigMatrix.Submatrix(Slice<4,8>(),Slice<4,8>()) = arr;

  // For all intents and purposes you can treat reference matrices as just regular matrices.
  RefMatrix<2,4,Array<8,8>> anotherRef = bigMatrix.Submatrix(Slice<2,4>(),Slice<1,5>()); // this creates a 2x4 reference matrix starting at element (2,1) of bigMatrix

  // You can fill them
  anotherRef.Fill(1.1111);

  // Do arithmetic with them
  anotherRef * bigMatrixRef;

  // Invert them
  Invert(bigMatrixRef);

  // Print them
  Serial << "bigMatrixRef: " << bigMatrixRef << "\n";

  // You can even make a reference to a reference matrix, do arithmetic with that and then print the result
  Serial  << "result of convoluted operation: " << (anotherRef += bigMatrixRef.Submatrix(Slice<0,2>(),Slice<0,4>())) << "\n";

  // The only thing that you can't (shouldn't) really do is operate on two matrix references whose underlying memory overlap,
  // particularly when doing matrix multiplication.

  // Lastly, let's look at what became of bigMatrix after all of this, you might be able to make out the values of bigMatrixRef and anotherRef in their respective areas of bigMatrix.
  Serial << "bigMatrix: " << bigMatrix << "\n";
}

void loop() { }
