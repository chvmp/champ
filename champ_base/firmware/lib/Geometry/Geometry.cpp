#include "Geometry.h"

Point Point::CrossProduct(Point &p)
{
    Point ret;

    ret.X() = (*this).Y() * p.Z() - (*this).Z() * p.Y();
    ret.Y() = (*this).Z() * p.X() - (*this).X() * p.Z();
    ret.Z() = (*this).X() * p.Y() - (*this).Y() * p.X();

    return ret;
}

float Point::Magnitude()
{
    float ret = 0;

    for(int i = 0; i < 3; i++)
        ret += pow((*this)(i),2);

    return sqrtf(ret);
}

float Point::DotProduct(Point &obj)
{
    float sum = 0;

    for(int i = 0; i < 3; i++)
        sum += (*this)(i) * obj(i);

    return sum;
}

Rotation &Rotation::FromEulerAngles(float psi, float theta, float phi)
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

Matrix<3,2> Rotation::ToEulerAngles()
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

Rotation &Rotation::RotateX(float phi)
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

Rotation &Rotation::RotateY(float theta)
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

Rotation &Rotation::RotateZ(float psi)
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

Transformation &Transformation::operator*=(Transformation &obj)
{
    p.Matrix<3>::operator=(R * obj.p + p);
    R.Matrix<3,3>::operator=(R * obj.R);

    return *this;
}

Transformation Transformation::operator*(Transformation &obj)
{
    Transformation ret;

    ret.p.Matrix<3>::operator=(R * obj.p + p);
    ret.R.Matrix<3,3>::operator=(R * obj.R);

    return ret;
}

float &Transformation::operator()(int row, int col)
{
    static float dummy;

    if(col == 3)
        return (row == 3)? (dummy = 1) : p(row);
    else
        return (row == 3)? (dummy = 0) : R(row,col);
}

Transformation &Transformation::RotateX(float phi)
{
    Point tmp;
    R.RotateX(phi);

    tmp.X() = p.X();
    tmp.Y() = cosf(phi) * p.Y() - sinf(phi) * p.Z();
    tmp.Z() = sinf(phi) * p.Y() + cosf(phi) * p.Z();

    p = tmp;

    return *this;
}

Transformation &Transformation::RotateY(float theta)
{
    Point tmp;
    R.RotateY(theta);

    tmp.X() = cosf(theta) * p.X() + sinf(theta) * p.Z();
    tmp.Y() = p.Y();
    tmp.Z() = -sinf(theta) * p.X() + cosf(theta) * p.Z();

    p = tmp;

    return *this;
}

Transformation &Transformation::RotateZ(float psi)
{
    Point tmp;
    R.RotateZ(psi);

    tmp.X() = cosf(psi) * p.X() - sinf(psi) * p.Y();
    tmp.Y() = sinf(psi) * p.X() + cosf(psi) * p.Y();
    tmp.Z() = p.Z();

    p = tmp;

    return *this;
}


Transformation &Transformation::Translate(float x, float y, float z)
{
    (*this).p(0) += x;
    (*this).p(1) += y;
    (*this).p(2) += z;

    return (*this);
}

// Print &operator<<(Print &strm, const Point &obj)
// {
//     strm << (const Matrix<3,1>&)obj;
//     return strm;
// }

// Print &operator<<(Print &strm, const Rotation &obj)
// {
//     strm << (const Matrix<3,3>&)obj;
//     return strm;
// }

// // Stream inserter operator for printing to strings or the serial port
// Print &operator<<(Print &strm, const Transformation &obj)
// {
//     strm << '{';

//     for(int i = 0; i < 4; i++)
//     {
//         strm << '{';

//         for(int j = 0; j < 4; j++)
//         {
//             if(j == 3)
//                 strm << ((i == 3)? 1 : obj.p(i,j));
//             else
//                 strm << ((i == 3)? 0 : obj.R(i,j));

//             strm << ((j == 4 - 1)? '}' : ',');
//         }

//         strm << (i == 4 - 1? '}' : ',');
//     }
//     return strm;
// }

