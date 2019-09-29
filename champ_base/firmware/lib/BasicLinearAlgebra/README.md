# Basic Linear Algebra

Is a library for representing matrices and doing matrix math on arduino. It provides a Matrix class which can be used to declare 2D matrices of arbitrary height, width, type and even storage policy (see below). The matrix overrides the +, +=, -, -=, *, *= and = operators so they can be used naturally in algebraic expressions. It also supports a few more advanced operations including matrix inversion. It doesn't use malloc so it's not prone to leaks or runtime errors and it even checks whether the dimensions of two matrices used in an expression are compatible at compile time (so it'll let you know when you try to add together two matrices with different dimensions for example).

## How it Works

### The Basics
This library defines a class Matrix which you can use to represent two dimensional matrices of arbitrary dimensions. 

For example, you can declare a 3x2 matrix like so:
```
Matrix<3,2> A;
```
Or a 6x3 Matrix like so:
```
Matrix<6,3> B;
```
You can do lots of different things with matrices including basic algebra:
```
Matrix<6,2> C = B * A;
```
As well as some more advanced operations like Transposition, Concatenation and Inversion. For more on the basic useage of matrices have a look at the [HowToUse](https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/HowToUse/HowToUse.ino) example.

### Changing the Element Type

By default, matrices are full of floats. If that doesn't suit your application, then you can specify what kind of datatype you'd like your matrix to be made of by passing it in to the class's parameters just after the dimensions like so:
```
Matrix<3,2,Array<3,2,int> > D;
```
Not only that, the datatype needn't be just a POD (float, bool, int etc) but it can be any class or struct you define too. So if you'd like to do matrix math with your own custom class then you can, so long as it implements the operators for addition, subtraction etc. For example, if you wanted, you could write a class to hold an imaginary number and then you could make a matrix out of it like so:
```
Matrix<2,2,Array<2,2,ImaginaryNumber> > Im;
```
This turned out to have intersting implications when you specify the element type as another matrix. For more on that, have a look at the [Tensor](https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/Tensor/Tensor.ino) example.

### Changing the Storage Policy

On a loosely related note, by default, matrices store their elements in an C-style array which is kept inside the object. Every time you access one of the elements directly using the () operator or indirectly when you do some algebra, the matrix returns the appropriate element from that array. 

At times, it's handy to be able to instruct the matrix to do something else when asked to retrieve an element at a particular index. For example if you know that your matrix is very sparse (only a few elements are non-zero) then you can save yourself some memory and just store the non-zero elements and return zero when asked for anything else.

If you like, you can override the way in which the elements for the matrix are stored by passing something that I've called a memory delegate into the Matrix class. For example, we can define a 2000x3000 matrix with a sparse storage policy like so:  
```
Matrix<2000,3000, float, Sparse<3000,100,float> > sparseMatrix;
```
In this case the ```Sparse<3000,100,float>``` type is the memory delegate which provides storage for 100 elements which are assumed to be embedded in a matrix having 3000 columns. You can find the implementation for this class in MemoryDelegate.hpp file, but long story short, it's a hashmap.

You can implement the memory delegate in whatever way you like so long as it returns some piece of memory when passed a row/column index. For more on how to implement a memory delegate, have a look at the  [CustomMatrix](https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/CustomMatrix/CustomMatrix.ino) example.

### Reference Matrices

One particularly useful part of being able to override the way matrices access their elements is that it lets us define reference matrices. Reference matrices don't actually own any memory themselves, instead they return the elements of another matrix when we access them. To create a reference matrix you can use the Submatrix method of the matrix class like so:
```
auto ref = A.Submatrix(Slice<1,3>(),Slice<0,2>()));
```
Here, ref is a 2x2 matrix which returns the elements in the lower 2 rows of the 3x2 matrix A defined above. I've let the compiler infer the type of ref using the 'auto' keyword but if we were to write it out ourselves it'd be:
```
Matrix<2,2,float, Ref<float,Array<3,2> >;
```
Where Ref<float,Array<3,2> is a memory delegate which takes a reference to a 3x2 matrix whose own memory delegate is an Array (the default) of size 3x2.

In general, reference matrices are useful for isolating a subsection of a larger matrix. That lets us use just that section in matrix operations with other matrices of compatible dimensions, or to collectively assign a value to a particular section of a matrix. For more information on reference matrices check out the [References](https://github.com/tomstewart89/BasicLinearAlgebra/blob/master/examples/References/References.ino) example.
