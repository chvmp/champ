#include <BasicLinearAlgebra.h>

/* 
 * Many matrices have special properties which mean that they can be represented with less memory or that computations that they are involved can be carried out much more efficiently.
 * Sparse matrices for example might have enormous dimensions but only a few non-zero elements. So in these kinds of matrices it's a waste to reserve a huge amount of memory for a
 * whole heap of zeros. Similarly, it's a waste to cycle through each of those zeros every time we want to do a computation. Instead it'd be better to just store the non-zero elements
 * and skip past the zeros when doing a computation.
 * 
 * For that reason, I've written the matrix class such that it's memory and the way it is accessed can be customised to take advantage of whatever helpful properties the particular
 * matrix might have. In this example we'll look at a diagonal matrix - a matrix whose elements are zero except for those along it's diagonal (row == column) 
 */

template<int dim, class ElemT> struct Diagonal
{
    mutable ElemT m[dim];

    // The only requirement on this class is that it implement the () operator like so:
    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const
    {
        static ElemT dummy;

        // If it's on the diagonal and it's not larger than the matrix dimensions then return the element
        if(row == col && row < dim)
            return m[row];
        else
            // Otherwise return a zero
            return (dummy = 0);
    }
};

// All the functions in BasicLinearAlgebra are wrapped up inside the namespace BLA, so specify that we're using it like so:
using namespace BLA;

void setup()
{
  Serial.begin(115200);

  // If you've been through the HowToUse example you'll know that you can allocate a Matrix and explicitly specify it's type like so:
  BLA::Matrix<4,4> mat;

  // And as before it's a good idea to fill the matrix before we use it
  mat.Fill(1);

  // Now let's declare a diagonal matrix. To do that we pass the Diagonal class from above along with whatever template parameters
  // as a template parameter to Matrix, like so:
  BLA::Matrix<4, 4, Diagonal<4, float> > diag;

  // If we fill diag we'll get a matrix with all 1's along the diagonal, the identity matrix.
  diag.Fill(1);

  // So multiplying it with mat will do nothing:
  Serial << "still ones: " << diag * mat << "\n";

  // Diagonal matrices have the handy property of scaling either the rows (premultiplication) or columns (postmultiplication) of a matrix

  // So if we modify the diagonal
  for(int i = 0; i < diag.GetRowCount(); i++)
      diag.delegate(i,i) = i + 1;

  // And multiply again, we'll see that the rows have been scaled
  Serial << "scaled rows: " << diag * mat;

  // Point being, if you define a class which serves up something when called upon by the () operator, you can embed it in a matrix and
  // define any kind of behaviour you like. Hopefully that'll let this library support lots more applications while catering to the
  // arduino's limited amount of memory.
}

void loop() { }
