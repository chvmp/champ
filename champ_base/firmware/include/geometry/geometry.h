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

#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "bla/basic_linear_algebra.h"

#include <math.h>

using namespace BLA;

// A point class for representing coordinates in a 3 dimensional space
namespace geometry
{
    class Point : public Matrix<3,1>
    {
        public:
            Point() { Fill(0); }
            Point(const Point &obj) : Matrix<3,1>() { (*this) = obj; }
            Point(const Matrix<3,1> &obj) { (*this) = obj; }

            float Magnitude()
            {
                float ret = 0;

                for(int i = 0; i < 3; i++)
                    ret += pow((*this)(i),2);

                return sqrtf(ret);
            }

            float DotProduct(Point &obj)
            {
                float sum = 0;

                for(int i = 0; i < 3; i++)
                    sum += (*this)(i) * obj(i);

                return sum;
            }

            Point CrossProduct(Point &p)
            {
                Point ret;

                ret.X() = (*this).Y() * p.Z() - (*this).Z() * p.Y();
                ret.Y() = (*this).Z() * p.X() - (*this).X() * p.Z();
                ret.Z() = (*this).X() * p.Y() - (*this).Y() * p.X();

                return ret;
            }

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

            Rotation &FromEulerAngles(float psi, float theta, float phi)
            {
                (*this)(0,0) = cosf(phi) * cosf(theta);
                (*this)(1,0) = cosf(theta) * sinf(phi);
                (*this)(2,0) = -sinf(theta);

                (*this)(0,1) = cosf(phi) * sinf(psi) * sinf(theta) - cosf(psi) * sinf(phi);
                (*this)(1,1) = cosf(psi) * cosf(phi) + sinf(psi) * sinf(phi) * sinf(theta);
                (*this)(2,1) = cosf(theta) * sinf(psi);

                (*this)(0,2) = sinf(psi) * sinf(phi) + cosf(psi) * cosf(phi) * sinf(theta);
                (*this)(1,2) = cosf(psi) * sinf(phi) * sinf(theta) - cosf(phi) * sinf(psi);
                (*this)(2,2) = cosf(psi) * cosf(theta);

                return (*this);
            }    

            Matrix<3,2> ToEulerAngles()
            {
                Matrix<3,2> ret;

                if((*this)(2,0) != 0)
                {
                    ret(1,0) = -asinf((*this)(2,0));
                    ret(1,1) = M_PI - ret(1,0);

                    ret(0,0) = atan2f((*this)(2,1) / cosf(ret(1,0)),(*this)(2,2) / cosf(ret(1,0)));
                    ret(0,1) = atan2f((*this)(2,1) / cosf(ret(1,1)),(*this)(2,2) / cosf(ret(1,1)));

                    ret(2,0) = atan2f((*this)(1,0) / cosf(ret(1,0)),(*this)(0,0) / cosf(ret(1,0)));
                    ret(2,1) = atan2f((*this)(1,0) / cosf(ret(1,1)),(*this)(0,0) / cosf(ret(1,1)));
                }
                else
                {
                    ret(2,0) = ret(2,1) = 0;

                    if((*this)(2,0) == -1)
                    {
                        ret(1,0) = ret(1,1) = M_PI_2;
                        ret(0,0) = ret(0,1) = atan2f((*this)(0,1),(*this)(0,2));
                    }
                    else
                    {
                        ret(1,0) = ret(1,1) = -M_PI_2;
                        ret(0,0) = ret(0,1) = atan2f(-(*this)(0,1),-(*this)(0,2));
                    }
                }

                return ret;
            }

            Rotation &RotateX(float phi)
            {
                float tmp1, tmp2;

                tmp1 = (*this)(1,0) * cosf(phi) - (*this)(2,0) * sinf(phi);
                tmp2 = (*this)(2,0) * cosf(phi) + (*this)(1,0) * sinf(phi);
                (*this)(1,0) = tmp1;
                (*this)(2,0) = tmp2;

                tmp1 = (*this)(1,1) * cosf(phi) - (*this)(2,1) * sinf(phi);
                tmp2 = (*this)(2,1) * cosf(phi) + (*this)(1,1) * sinf(phi);
                (*this)(1,1) = tmp1;
                (*this)(2,1) = tmp2;

                tmp1 = (*this)(1,2) * cosf(phi) - (*this)(2,2) * sinf(phi);
                tmp2 = (*this)(2,2) * cosf(phi) + (*this)(1,2) * sinf(phi);
                (*this)(1,2) = tmp1;
                (*this)(2,2) = tmp2;

                return (*this);
            }    
            
            Rotation &RotateY(float theta)
            {
                float tmp1, tmp2;

                tmp1 = (*this)(0,0) * cosf(theta) + (*this)(2,0) * sinf(theta);
                tmp2 = (*this)(2,0) * cosf(theta) - (*this)(0,0) * sinf(theta);
                (*this)(0,0) = tmp1;
                (*this)(2,0) = tmp2;

                tmp1 = (*this)(0,1) * cosf(theta) + (*this)(2,1) * sinf(theta);
                tmp2 = (*this)(2,1) * cosf(theta) - (*this)(0,1) * sinf(theta);
                (*this)(0,1) = tmp1;
                (*this)(2,1) = tmp2;

                tmp1 = (*this)(0,2) * cosf(theta) + (*this)(2,2) * sinf(theta);
                tmp2 = (*this)(2,2) * cosf(theta) - (*this)(0,2) * sinf(theta);
                (*this)(0,2) = tmp1;
                (*this)(2,2) = tmp2;

                return (*this);
            }

            Rotation &RotateZ(float psi)
            {
                float tmp1, tmp2;

                tmp1 = (*this)(0,0) * cosf(psi) -  (*this)(1,0) * sinf(psi);
                tmp2 = (*this)(1,0) * cosf(psi) +  (*this)(0,0) * sinf(psi);
                (*this)(0,0) = tmp1;
                (*this)(1,0) = tmp2;

                tmp1 = (*this)(0,1) * cosf(psi) -  (*this)(1,1) * sinf(psi);
                tmp2 = (*this)(1,1) * cosf(psi) +  (*this)(0,1) * sinf(psi);
                (*this)(0,1) = tmp1;
                (*this)(1,1) = tmp2;


                tmp1 = (*this)(0,2) * cosf(psi) -  (*this)(1,2) * sinf(psi);
                tmp2 = (*this)(1,2) * cosf(psi) +  (*this)(0,2) * sinf(psi);
                (*this)(0,2) = tmp1;
                (*this)(1,2) = tmp2;

                return (*this);
            }
            
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

            Transformation operator*=(Transformation &obj)
            {
                p.Matrix<3>::operator=(R * obj.p + p);
                R.Matrix<3,3>::operator=(R * obj.R);

                return *this;
            }

            Transformation operator*(Transformation &obj)
            {
                Transformation ret;

                ret.p.Matrix<3>::operator=(R * obj.p + p);
                ret.R.Matrix<3,3>::operator=(R * obj.R);

                return ret;
            }

            float &operator()(int row, int col)
            {
                static float dummy;

                if(col == 3)
                    return (row == 3)? (dummy = 1) : p(row);
                else
                    return (row == 3)? (dummy = 0) : R(row,col);
            }

            float &X() { return p(0); }
            float &Y() { return p(1); }
            float &Z() { return p(2); }

            Transformation &RotateX(float phi)
            {
                Point tmp;
                R.RotateX(phi);

                tmp.X() = p.X();
                tmp.Y() = cosf(phi) * p.Y() - sinf(phi) * p.Z();
                tmp.Z() = sinf(phi) * p.Y() + cosf(phi) * p.Z();

                p = tmp;

                return *this;
            }

            Transformation &RotateY(float theta)
            {
                Point tmp;
                R.RotateY(theta);

                tmp.X() = cosf(theta) * p.X() + sinf(theta) * p.Z();
                tmp.Y() = p.Y();
                tmp.Z() = -sinf(theta) * p.X() + cosf(theta) * p.Z();

                p = tmp;

                return *this;
            }

            Transformation &RotateZ(float psi)
            {
                Point tmp;
                R.RotateZ(psi);

                tmp.X() = cosf(psi) * p.X() - sinf(psi) * p.Y();
                tmp.Y() = sinf(psi) * p.X() + cosf(psi) * p.Y();
                tmp.Z() = p.Z();

                p = tmp;

                return *this;
            }

            Transformation Translate(float x, float y, float z)
            {
                (*this).p(0) += x;
                (*this).p(1) += y;
                (*this).p(2) += z;

                return (*this);
            }

            template<class opMemT> Transformation &operator=(const Matrix<4,4,opMemT> &obj)
            {
                R = obj.Submatrix(Slice<0,3>(),Slice<0,3>());
                p = obj.Submatrix(Slice<0,3>(),Slice<3,4>());

                return *this;
            }
    };
}
#endif // GEOMETRY_H

