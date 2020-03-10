// MIT License

// Copyright (c) 2019 tomstewart89

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef BLA_H
#define BLA_H

#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "memory_delegate.h"

namespace BLA {

///////////////////////////////////////////////////////////////// Matrix ///////////////////////////////////////////////////////////////////

// Represents a Slice of rows or columns used in the () operator to select a submatrix
template<int start, int end> struct Slice { };

template<class MatrixT> class Inserter
{
    const MatrixT &parent;
    int index;

    void Insert(const typename MatrixT::mem_t::elem_t &val)
    {
        if(index < MatrixT::Rows * MatrixT::Cols)
        {
            parent(index / MatrixT::Cols, index % MatrixT::Cols) = val;
            ++index;
        } // else silently ignore the extra initialisers (todo: assert here)
    }

public:
    Inserter(const MatrixT &_parent, const typename MatrixT::mem_t::elem_t &val) : parent(_parent), index(0)
    {
        Insert(val);
    }

    Inserter<MatrixT> &operator,(const typename MatrixT::mem_t::elem_t &val)
    {
        Insert(val);
        return *this;
    }
};

template<int rows, int cols = 1, class MemT = Array<rows,cols,float> > class Matrix
{
public:
    typedef MemT mem_t;
    const static int Rows = rows;
    const static int Cols = cols;

    MemT delegate;

    // Constructors
    Matrix<rows,cols,MemT>() { }
    Matrix<rows,cols,MemT>(MemT &d) : delegate(d) { }
    Matrix<rows,cols,MemT>(typename MemT::elem_t arr[rows][cols]) { *this = arr; }
    template<typename ...ARGS> Matrix(ARGS... args) { FillRowMajor(args...); }

    template<class opMemT>
    Matrix<rows,cols,MemT>(const Matrix<rows,cols,opMemT> &obj) { *this = obj; }

    // Dimension Access
    int GetRowCount() const;
    int GetColCount() const;

    // Element Access
    typename MemT::elem_t &operator()(int row, int col = 0) const;
    template<int rowStart, int rowEnd, int colStart, int colEnd> Matrix<rowEnd-rowStart,colEnd-colStart,Reference<MemT> > Submatrix(Slice<rowStart,rowEnd>, Slice<colStart,colEnd>) const;
    Matrix<rows,cols,Reference<MemT> > Ref() const;

    // Concatenation
    template<int operandCols, class opMemT> Matrix<rows,cols + operandCols,HorzCat<cols,MemT,opMemT> > operator||(const Matrix<rows,operandCols,opMemT> &obj) const;
    template<int operandRows, class opMemT> Matrix<rows + operandRows,cols,VertCat<rows,MemT,opMemT> > operator&&(const Matrix<operandRows,cols,opMemT> &obj) const;

    // Assignment
    template<class opMemT> Matrix<rows,cols,MemT> &operator=(const Matrix<rows,cols,opMemT> &obj);
    Matrix<rows,cols,MemT> &operator=(typename MemT::elem_t arr[rows][cols]);
    Inserter<Matrix<rows,cols,MemT> > operator<<(const typename MemT::elem_t &val);
    Matrix<rows,cols,MemT> &Fill(const typename MemT::elem_t &val);
    template<typename ...TAIL> void FillRowMajor(typename MemT::elem_t head, TAIL... tail);
    void FillRowMajor() { }

    // Addition
    template<class opMemT> Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > operator+(const Matrix<rows,cols,opMemT> &obj) const;
    template<class opMemT> Matrix<rows,cols,MemT> &operator+=(const Matrix<rows,cols,opMemT> &obj);

    // Subtraction
    template<class opMemT> Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > operator-(const Matrix<rows,cols,opMemT> &obj) const;
    template<class opMemT> Matrix<rows,cols,MemT> &operator-=(const Matrix<rows,cols,opMemT> &obj);

    // Multiplication
    template <int operandCols, class opMemT> Matrix<rows,operandCols,Array<rows,operandCols,typename MemT::elem_t> > operator*(const Matrix<cols,operandCols,opMemT> &operand) const;
    template<class opMemT> Matrix<rows,cols,MemT> &operator*=(const Matrix<rows,cols,opMemT> &operand);

    // Negation
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > operator-() const;

    // Transposition
    Matrix<cols,rows,Trans<MemT> > operator~() const;

    // Elementwise Operations
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > operator+(const typename MemT::elem_t k) const;
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > operator-(const typename MemT::elem_t k) const;
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > operator*(const typename MemT::elem_t k) const;
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > operator/(const typename MemT::elem_t k) const;

    Matrix<rows,cols,MemT> &operator+=(const typename MemT::elem_t k);
    Matrix<rows,cols,MemT> &operator-=(const typename MemT::elem_t k);
    Matrix<rows,cols,MemT> &operator*=(const typename MemT::elem_t k);
    Matrix<rows,cols,MemT> &operator/=(const typename MemT::elem_t k);

    // Returns the inverse of this matrix - only supports square matrices
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Inverse(int *res = NULL) const;

    // Returns the determinant of this matrix
    typename MemT::elem_t Det() const { return Determinant(*this); }
};

//////////////////////////////////////////////////////////////// Element Access ////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
typename MemT::elem_t &Matrix<rows,cols,MemT>::operator()(int row, int col) const
{
    return delegate(row,col);
}

// this must have template arguments so that it can return a matrix of he right size, right? can they be inferred somehow>
template<int rows, int cols, class MemT>
template<int rowStart, int rowEnd, int colStart, int colEnd>
Matrix<rowEnd-rowStart,colEnd-colStart,Reference<MemT> > Matrix<rows,cols,MemT>::Submatrix(Slice<rowStart,rowEnd>, Slice<colStart,colEnd>) const
{
    Reference<MemT> ref(delegate, rowStart, colStart);
    return Matrix<rowEnd-rowStart,colEnd-colStart,Reference<MemT> >(ref);
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,Reference<MemT> > Matrix<rows,cols,MemT>::Ref() const
{
    Reference<MemT> ref(delegate, 0,0);
    return Matrix<rows,cols,Reference<MemT> >(ref);
}

///////////////////////////////////////////////////////////////// Concatenation ////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
template<int operandCols, class opMemT>
Matrix<rows,cols+operandCols,HorzCat<cols,MemT,opMemT> > Matrix<rows,cols,MemT>::operator||(const Matrix<rows,operandCols,opMemT> &obj) const
{
    HorzCat<cols,MemT,opMemT> ref(delegate, obj.delegate);
    return Matrix<rows,cols+operandCols,HorzCat<cols,MemT,opMemT> >(ref);
}

template<int rows, int cols, class MemT>
template<int operandRows, class opMemT>
Matrix<rows+operandRows,cols,VertCat<rows,MemT,opMemT> > Matrix<rows,cols,MemT>::operator&&(const Matrix<operandRows,cols,opMemT> &obj) const
{
    VertCat<rows,MemT,opMemT> ref(delegate, obj.delegate);
    return Matrix<rows+operandRows,cols,VertCat<rows,MemT,opMemT> >(ref);
}

///////////////////////////////////////////////////////////////// Assignment ///////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
template<class opMemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator=(const Matrix<rows,cols,opMemT> &obj)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            (*this)(i,j)  = obj(i,j);

    return *this;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator=(typename MemT::elem_t arr[rows][cols])
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            (*this)(i,j)  = arr[i][j];

    return *this;
}

template<int rows, int cols, class MemT>
Inserter<Matrix<rows,cols,MemT> > Matrix<rows,cols,MemT>::operator<<(const typename MemT::elem_t &val)
{
    return Inserter<Matrix<rows,cols,MemT> >(*this,val);
}


template<int rows, int cols, class MemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::Fill(const typename MemT::elem_t &val)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            (*this)(i,j) = val;

    return *this;
}

template<int rows, int cols, class MemT>
template<typename ...TAIL>
void Matrix<rows,cols,MemT>::FillRowMajor(typename MemT::elem_t head, TAIL... tail)
{
    static_assert(rows*cols > sizeof...(TAIL), "Too many arguments passed to FillRowMajor");
    (*this)((rows*cols - sizeof...(TAIL) - 1) / cols,(rows*cols - sizeof...(TAIL) - 1) % cols) = head;
    FillRowMajor(tail...);
}

//////////////////////////////////////////////////////////////// Dimension Access ////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
int Matrix<rows,cols,MemT>::GetRowCount() const
{
    return rows;
}

template<int rows, int cols, class MemT>
int Matrix<rows,cols,MemT>::GetColCount() const
{
    return cols;
}

////////////////////////////////////////////////////////////////// Addition ////////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
template<class opMemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator+(const Matrix<rows,cols,opMemT> &obj) const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret;
    Add(*this,obj,ret);

    return ret;
}

template<int rows, int cols, class MemT>
template<class opMemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator+=(const Matrix<rows,cols,opMemT> &obj)
{
    Add(*this,obj,*this);

    return *this;
}

template<int rows, int cols, class MemT, class opMemT, class retMemT>
Matrix<rows,cols,retMemT> &Add(const Matrix<rows,cols,MemT> &A, const Matrix<rows,cols,opMemT> &B, Matrix<rows,cols,retMemT> &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C.delegate(i,j) = A.delegate(i,j) + B.delegate(i,j);

    return C;
}

////////////////////////////////////////////////////////////////// Subtraction /////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
template<class opMemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator-(const Matrix<rows,cols,opMemT> &obj) const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret;
    Subtract(*this,obj,ret);

    return ret;
}

template<int rows, int cols, class MemT>
template<class opMemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator-=(const Matrix<rows,cols,opMemT> &obj)
{
    Subtract(*this,obj,*this);

    return *this;
}

template<int rows, int cols, class MemT, class opMemT, class retMemT>
Matrix<rows,cols,retMemT> &Subtract(const Matrix<rows,cols,MemT> &A, const Matrix<rows,cols,opMemT> &B, Matrix<rows,cols,retMemT> &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C(i,j) = A(i,j) - B(i,j);

    return C;
}

//////////////////////////////////////////////////////////////// Multiplication ////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
template <int operandCols, class opMemT>
Matrix<rows,operandCols,Array<rows,operandCols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator*(const Matrix<cols,operandCols,opMemT> &operand) const
{
    Matrix<rows,operandCols,Array<rows,operandCols,typename MemT::elem_t> > ret;
    Multiply(*this,operand,ret);

    return ret;
}

template<int rows, int cols, class MemT>
template <class opMemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator*=(const Matrix<rows,cols,opMemT> &operand)
{
    Matrix<rows,cols,MemT> ret;
    Multiply(*this,operand,ret);
    *this = ret;

    return *this;
}

// Multiplies two matrices and stores the result in a third matrix C, this is slightly faster than using the operators
template<int rows, int cols, int operandCols, class MemT, class opMemT, class retMemT>
Matrix<rows,operandCols,retMemT> &Multiply(const Matrix<rows,cols,MemT> &A, const Matrix<cols,operandCols,opMemT> &B, Matrix<rows,operandCols,retMemT> &C)
{
    int i,j,k;

    for(i = 0; i < rows; i++)
        for(j = 0; j < operandCols; j++)
        {
            if(cols > 0)
                C.delegate(i,j) = A.delegate(i,0) * B.delegate(0,j);

            for(k = 1; k < cols; k++)
                C.delegate(i,j) += A.delegate(i,k) * B.delegate(k,j);
        }

    return C;
}

/////////////////////////////////////////////////////////////////// Negation ///////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator-() const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret;

    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            ret(i,j) = -(*this)(i,j);

    return ret;
}

///////////////////////////////////////////////////////////////// Transposition ////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
Matrix<cols,rows,Trans<MemT> > Matrix<rows,cols,MemT>::operator~() const
{
    Trans<MemT> ref(delegate);
    Matrix<cols,rows,Trans<MemT> >tmp(ref);

    return tmp;
}

////////////////////////////////////////////////////////////// Elementwise Operations ////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator+(const typename MemT::elem_t k) const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret;
    ElementwiseAdd(*this,k,ret);

    return ret;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator+=(const typename MemT::elem_t k)
{
    ElementwiseAdd(*this,k,*this);

    return *this;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator-(const typename MemT::elem_t k) const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret;
    ElementwiseSubtract(*this,k,ret);

    return ret;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator-=(const typename MemT::elem_t k)
{
    ElementwiseSubtract(*this,k,*this);

    return *this;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator*(const typename MemT::elem_t k) const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret;
    ElementwiseMultiply(*this,k,ret);

    return ret;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator*=(const typename MemT::elem_t k)
{
    ElementwiseMultiply(*this,k,*this);

    return *this;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::operator/(const typename MemT::elem_t k) const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret;
    ElementwiseDivide(*this,k,ret);

    return ret;
}

template<int rows, int cols, class MemT>
Matrix<rows,cols,MemT> &Matrix<rows,cols,MemT>::operator/=(const typename MemT::elem_t k)
{
    ElementwiseDivide(*this,k,*this);

    return *this;
}

// Multiplies two matrices and stores the result in a third matrix C, this is slightly faster than using the operators
template<int rows, int cols, class MemT, class retMemT>
Matrix<rows,cols,retMemT> &ElementwiseAdd(const Matrix<rows,cols,MemT> &A, const typename MemT::elem_t &B, Matrix<rows,cols,retMemT> &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C(i,j) = A(i,j) + B;

    return C;
}

// Multiplies two matrices and stores the result in a third matrix C, this is slightly faster than using the operators
template<int rows, int cols, class MemT, class retMemT>
Matrix<rows,cols,retMemT> &ElementwiseSubtract(const Matrix<rows,cols,MemT> &A, const typename MemT::elem_t &B, Matrix<rows,cols,retMemT> &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C(i,j) = A(i,j) - B;

    return C;
}

// Multiplies two matrices and stores the result in a third matrix C, this is slightly faster than using the operators
template<int rows, int cols, class MemT, class retMemT>
Matrix<rows,cols,retMemT> &ElementwiseMultiply(const Matrix<rows,cols,MemT> &A, const typename MemT::elem_t &B, Matrix<rows,cols,retMemT> &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C(i,j) = A(i,j) * B;

    return C;
}

// Multiplies corresponding elements of two matrices  and stores result in third matrix C
template<int rows, int cols, class MemT, class opMemT, class retMemT>
Matrix<rows,cols,retMemT> &ElementwiseMultiply(const Matrix<rows,cols,MemT> &A, const Matrix<rows,cols,opMemT> &B, Matrix<rows,cols,retMemT> &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C(i,j) = A(i,j) * B(i,j);
	
    return C;
}
    
// Multiplies two matrices and stores the result in a third matrix C, this is slightly faster than using the operators
template<int rows, int cols, class MemT, class retMemT>
Matrix<rows,cols,retMemT> &ElementwiseDivide(const Matrix<rows,cols,MemT> &A, const typename MemT::elem_t &B, Matrix<rows,cols,retMemT> &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C(i,j) = A(i,j) / B;

    return C;
}

/////////////////////////////////////////////////////////////////// Determinant //////////////////////////////////////////////////////////////////

template<int dim, class MemT>
typename MemT::elem_t Determinant(const Matrix<dim,dim,MemT> &A)
{
    typename MemT::elem_t det = 0.0;
    for(int i = 0; i < dim; i++)
    {
        Minor<MemT> del(A.delegate, i, 0);
        Matrix<dim-1,dim-1,Minor<MemT> > m(del);
        
        // det += Determinant(m) * (i % 2? -A(i,0) : A(i,0));

        if (i%2)
            det -= Determinant(m) * A(i,0);
        else
            det += Determinant(m) * A(i,0);
    }

    return det;
}

template<class MemT>
typename MemT::elem_t Determinant(Matrix<2,2,MemT> &A)
{
    return A(0,0) * A(1,1) - A(1,0) * A(0,1);
}

/////////////////////////////////////////////////////////////////// Inversion //////////////////////////////////////////////////////////////////

template<int rows, int cols, class MemT>
Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > Matrix<rows,cols,MemT>::Inverse(int *res) const
{
    Matrix<rows,cols,Array<rows,cols,typename MemT::elem_t> > ret(*this);
    return Invert(ret, res);
}

// Matrix Inversion Routine - modified from code written by Charlie Matlack: http://playground.arduino.cc/Code/MatrixMath
// This function inverts a matrix based on the Gauss Jordan method. Specifically, it uses partial pivoting to improve numeric stability.
// The algorithm is drawn from those presented in NUMERICAL RECIPES: The Art of Scientific Computing.
template<int dim, class MemT>
Matrix<dim,dim,MemT> &Invert(Matrix<dim,dim,MemT> &A, int *res = NULL)
{
    int pivrow, pivrows[dim]; 	// keeps track of current pivot row and row swaps
    int i,j,k;
    typename MemT::elem_t tmp;		// used for finding max value and making column swaps

    for (k = 0; k < dim; k++)
    {
        // find pivot row, the row with biggest entry in current column
        tmp = 0;
        for (i = k; i < dim; i++)
        {
            if(fabs(A(i,k)) >= tmp)
            {
                tmp = fabs(A(i,k));
                pivrow = i;
            }
        }

        // check for singular matrix
        if (A(pivrow,k) == 0.0f)
            if(res)
                *res = 1;

        // Execute pivot (row swap) if needed
        if (pivrow != k)
        {
            // swap row k with pivrow
            for (j = 0; j < dim; j++)
            {
                tmp = A(k,j);
                A(k,j) = A(pivrow,j);
                A(pivrow,j) = tmp;
            }
        }
        pivrows[k] = pivrow;	// record row swap (even if no swap happened)

        tmp = 1.0f / A(k,k);	// invert pivot element
        A(k,k) = 1.0f;		// This element of input matrix becomes result matrix

        // Perform row reduction (divide every element by pivot)
        for (j = 0; j < dim; j++)
            A(k,j) = A(k,j) * tmp;

        // Now eliminate all other entries in this column
        for (i = 0; i < dim; i++)
        {
            if (i != k)
            {
                tmp = A(i,k);
                A(i,k) = 0.0f;  // The other place where in matrix becomes result mat

                for (j = 0; j < dim; j++)
                    A(i,j) = A(i,j) - A(k,j) * tmp;
            }
        }
    }

    // Done, now need to undo pivot row swaps by doing column swaps in reverse order
    for (k = dim-1; k >= 0; k--)
    {
        if (pivrows[k] != k)
        {
            for (i = 0; i < dim; i++)
            {
                tmp = A(i,k);
                A(i,k) = A(i,pivrows[k]);
                A(i,pivrows[k]) = tmp;
            }
        }
    }

    if(res)
        *res = 0;

    return A;
}

////////////////////////////////////////////////////////////////// Insertion ///////////////////////////////////////////////////////////////////

// inline Print &operator <<(Print &strm, const int obj)
// {
//     strm.print(obj); return strm;
// }

// inline Print &operator <<(Print &strm, const float obj)
// {
//     strm.print(obj); return strm;
// }

// inline Print &operator <<(Print &strm, const char *obj)
// {
//     strm.print(obj); return strm;
// }

// inline Print &operator <<(Print &strm, const char obj)
// {
//     strm.print(obj); return strm;
// }

// // Stream inserter operator for printing to strings or the serial port
// template<int rows, int cols, class MemT>
// Print &operator<<(Print &strm, const Matrix<rows,cols,MemT> &obj)
// {
//     strm << '{';

//     for(int i = 0; i < rows; i++)
//     {
//         strm << '{';

//         for(int j = 0; j < cols; j++)
//             strm << obj(i,j) << ((j == cols - 1)? '}' : ',');

//         strm << (i == rows - 1? '}' : ',');
//     }
//     return strm;
// }

} // namespace BLA

#endif // BLA_H
