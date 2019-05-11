#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Arduino.h"
#include "BasicLinearAlgebra.h"

#include <math.h>

using namespace BLA;

// A point class for representing coordinates in a 3 dimensional space
class Point : public Matrix<3,1>
{
public:
    Point() { Fill(0); }
    Point(const Point &obj) : Matrix<3,1>() { (*this) = obj; }
    Point(const Matrix<3,1> &obj) { (*this) = obj; }

    float Magnitude();
    float DotProduct(Point &obj);
    Point CrossProduct(Point &p);

    float &X() { return (*this)(0); }
    float &Y() { return (*this)(1); }
    float &Z() { return (*this)(2); }

    template<class opMemT> Point &operator=(const Matrix<3,1,opMemT> &obj)
    {
        for(int i = 0; i < 3; i++)
            (*this)(i,0) = obj(i,0);

        return *this;
    }
};

// A rotation matrix class in a 3 dimensional space
class Rotation : public Matrix<3,3>
{
public:
    Rotation() { *this = Identity<3,3>(); }
    Rotation(const Rotation &obj) : Matrix<3,3>() { (*this) = obj; }
    Rotation(const Matrix<3,3> &obj) { (*this) = obj; }

    Rotation &FromEulerAngles(float phi, float theta, float psi);
    Matrix<3,2> ToEulerAngles();

    Rotation &RotateX(float phi);
    Rotation &RotateY(float theta);
    Rotation &RotateZ(float psi);

    template<class opMemT> Rotation &operator=(const Matrix<3,3,opMemT> &obj)
    {
        for(int i = 0; i < Rows; i++)
            for(int j = 0; j < Cols; j++)
                (*this)(i,j)  = obj(i,j);

        return *this;
    }
};

// A transformation matrix class (rotation plus a coordinate) in a 3 dimensional space
class Transformation
{
public:
    Rotation R;
    Point p;

    Transformation() { R = Identity<3,3>(); p.Fill(0); }
    Transformation(const Transformation &obj) { (*this) = obj; }

    Transformation &operator*=(Transformation &obj);
    Transformation operator*(Transformation &obj);

    float &operator()(int row, int col);

    float &X() { return p(0); }
    float &Y() { return p(1); }
    float &Z() { return p(2); }

    Transformation &RotateX(float phi);
    Transformation &RotateY(float theta);
    Transformation &RotateZ(float psi);

    Transformation &Translate(float x, float y, float z);

    template<class opMemT> Transformation &operator=(const Matrix<4,4,opMemT> &obj)
    {
        R = obj.Submatrix(Slice<0,3>(),Slice<0,3>());
        p = obj.Submatrix(Slice<0,3>(),Slice<3,4>());

        return *this;
    }
};

// Stream inserters operator for printing to strings or the serial port
Print &operator<<(Print &strm, const Point &obj);
Print &operator<<(Print &strm, const Rotation &obj);
Print &operator<<(Print &strm, const Transformation &obj);

#endif // GEOMETRY_H

