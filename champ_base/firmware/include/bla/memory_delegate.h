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

#ifndef MEMORY_DELEGATE_H
#define MEMORY_DELEGATE_H

namespace BLA {

template<int rows, int cols, class MemT> class Matrix;

///////////////////////////////////////////////////////////////// Array Memory Delegate ///////////////////////////////////////////////////////////////////

template<int rows, int cols = 1, class ElemT = float> struct Array
{
    typedef ElemT elem_t;
    mutable elem_t m[rows * cols];

    ElemT &operator()(int row, int col) const
    {
        static elem_t dummy;

        if(row > rows || col > cols)
            return dummy;
        else
            return m[row * cols + col];
    }
};

template<int rows, int cols, class ElemT, class opElemT, class retElemT>
Matrix<rows,cols,Array<rows,cols,retElemT> > &Add(const Matrix<rows,cols,Array<rows,cols,ElemT> > &A, const Matrix<rows,cols,Array<rows,cols,opElemT> > &B, Matrix<rows,cols,Array<rows,cols,retElemT> > &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C.delegate.m[i * cols + j] = A.delegate.m[i * cols + j] + B.delegate.m[i * cols + j];

    return C;
}

template<int rows, int cols, class ElemT, class opElemT, class retElemT>
Matrix<rows,cols,Array<rows,cols,retElemT> > &Subtract(const Matrix<rows,cols,Array<rows,cols,ElemT> > &A, const Matrix<rows,cols,Array<rows,cols,opElemT> > &B, Matrix<rows,cols,Array<rows,cols,retElemT> > &C)
{
    for(int i = 0; i < rows; i++)
        for(int j = 0; j < cols; j++)
            C.delegate.m[i * cols + j] = A.delegate.m[i * cols + j] - B.delegate.m[i * cols + j];

    return C;
}

template<int rows, int cols, int operandCols, class ElemT, class opElemT, class retElemT>
Matrix<rows,operandCols,Array<rows,operandCols,retElemT> > &Multiply(const Matrix<rows,cols,Array<rows,cols,ElemT> > &A, const Matrix<cols,operandCols,Array<cols,operandCols,opElemT> > &B, Matrix<rows,operandCols,Array<rows,operandCols,retElemT> > &C)
{
    int i,j,k;

    for(i = 0; i < rows; i++)
        for(j = 0; j < operandCols; j++)
        {
            if(cols > 0)
                C.delegate.m[i * operandCols + j] = A.delegate.m[i * cols] * B.delegate.m[j];

            for(k = 1; k < cols; k++)
                C.delegate.m[i * operandCols + j] += A.delegate.m[i * cols + k] * B.delegate.m[k * operandCols + j];
        }

    return C;
}

template<int rows, int cols = 1, class ElemT = float> using ArrayMatrix = Matrix<rows,cols,Array<rows,cols,ElemT> >;

///////////////////////////////////////////////////////////////// Reference Memory Delegate ///////////////////////////////////////////////////////////////////

template<class MemT> struct Reference
{
    typedef typename MemT::elem_t elem_t;

    const MemT &parent;
    int rowOffset, colOffset;

    Reference<MemT>(const MemT &obj, int rowOff, int colOff) : parent(obj), rowOffset(rowOff), colOffset(colOff) { }
    Reference<MemT>(const Reference<MemT> &obj) : parent(obj.parent), rowOffset(obj.rowOffset), colOffset(obj.colOffset) { }

    typename MemT::elem_t &operator()(int row, int col) const
    {
        return parent(row+rowOffset, col+colOffset);
    }
};

template<int rows, int cols, class ElemT = float> using ArrayRef = Reference<Array<rows,cols,ElemT> >;
template<int rows, int cols, class ParentMemT > using RefMatrix = Matrix<rows,cols,Reference<ParentMemT> >;


///////////////////////////////////////////////////////////////// Identity Memory Delegate ///////////////////////////////////////////////////////////////////

template<class ElemT> struct Iden
{
    typedef ElemT elem_t;

    elem_t &operator()(int row, int col) const
    {
        static elem_t ret;

        if(row == col)
            return (ret = 1);
        else
            return (ret = 0);
    }
};

template<int rows, int cols = rows, class ElemT = float> using Identity = Matrix<rows,cols,Iden<ElemT> >;

///////////////////////////////////////////////////////////////// Zeros Memory Delegate ///////////////////////////////////////////////////////////////////

template<class ElemT> struct Zero
{
    typedef ElemT elem_t;

    ElemT &operator()(int row, int col) const
    {
        static ElemT ret;

        return (ret = 0);
    }
};

template<int rows, int cols = 1, class ElemT = float> using Zeros = Matrix<rows,cols,Zero<ElemT> >;


///////////////////////////////////////////////////////////////// Sparse Memory Delegate ///////////////////////////////////////////////////////////////////

// This uses a hash table to look up row/col/val items. It uses an open addressing collision strategy so we can avoid using dynamic memory
template<int cols, int tableSize, class ElemT> struct Sparse
{
    typedef ElemT elem_t;

    struct HashItem
    {
        mutable int key;
        mutable ElemT val;

        HashItem() { key = -1; }

    } table[tableSize];

    ElemT &operator()(int row, int col) const
    {
        // Make a key out of the row / column
        int key = row * cols + col;

        // Calculate the hash by taking the modulo of the key with the tableSize
        int hash = key % tableSize;

        const HashItem *item;

        // Find a item with a key matching the input key
        for(int i = 0; i < tableSize; i++)
        {
            item = table + (hash + i) % tableSize;

            // If the element is empty or unused (val == 0) then the item doesn't exist in the table
            if(item->key == -1 || item->val == 0)
            {
                item->key = key;
                item->val = 0;
                break;
            }

            // If it's key matches the input key then return it
            if(item->key == key)
            {
                break;
            }
        }

        // If we landed on a matching key then we're done!
        if(item->key == key)
        {
            return item->val;
        }
        else
        {
            static ElemT outOfMemory;
            return outOfMemory;
        }
    }
};

template<int rows, int cols, int tableSize = cols, class ElemT = float> using SparseMatrix = Matrix<rows,cols,Sparse<cols,tableSize,ElemT> >;

//////////////////////////////////////////////////////////// Matrix Minor Memory Delegate ////////////////////////////////////////////////////////////////

template<class MemT> struct Minor
{
    typedef typename MemT::elem_t elem_t;
    const MemT parent;
    int i, j;

    Minor<MemT>(const MemT &obj, int row, int col) : parent(obj), i(row), j(col) { }

    elem_t &operator()(int row, int col) const
    {
        if(row >= i) row++;
        if(col >= j) col++;

        return parent(row,col);
    }
};

//////////////////////////////////////////////////////////// Transpose Delegate ////////////////////////////////////////////////////////////////

template<class MemT> struct Trans
{
    typedef typename MemT::elem_t elem_t;
    const MemT parent;

    Trans<MemT>(const MemT &obj) : parent(obj) { }
    Trans<MemT>(const Trans<MemT> &obj) : parent(obj.parent) { }

    elem_t &operator()(int row, int col) const
    {
        return parent(col,row);
    }
};

////////////////////////////////////////////////////////// Concatenation Delegates /////////////////////////////////////////////////////////////

template<int leftCols, class LeftMemT, class RightMemT> struct HorzCat
{
    typedef typename LeftMemT::elem_t elem_t;
    const LeftMemT left;
    const RightMemT right;

    HorzCat<leftCols,LeftMemT,RightMemT>(const LeftMemT &l, const RightMemT &r) : left(l), right(r) { }
    HorzCat<leftCols,LeftMemT,RightMemT>(const HorzCat<leftCols,LeftMemT,RightMemT> &obj) : left(obj.left), right(obj.right) { }

    virtual ~HorzCat<leftCols,LeftMemT,RightMemT>() { }

    elem_t &operator()(int row, int col) const
    {
        return col < leftCols? left(row,col) : right(row,col-leftCols);
    }
};

template<int topRows, class TopMemT, class BottomMemT> struct VertCat
{
    typedef typename TopMemT::elem_t elem_t;
    const TopMemT top;
    const BottomMemT bottom;

    VertCat<topRows,TopMemT,BottomMemT>(const TopMemT &t, const BottomMemT &b) : top(t), bottom(b) { }
    VertCat<topRows,TopMemT,BottomMemT>(const VertCat<topRows,TopMemT,BottomMemT> &obj) : top(obj.top), bottom(obj.bottom) { }

    virtual ~VertCat<topRows,TopMemT,BottomMemT>() { }

    elem_t &operator()(int row, int col) const
    {
        return row < topRows? top(row,col) : bottom(row-topRows,col);
    }
};

} // namespace BLA

#endif // MEMORY_DELEGATE_H
