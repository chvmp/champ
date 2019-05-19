#include <link.h>

class RevoluteJoint : public Link
{
    public:
        RevoluteJoint(float d, float theta, float r, float alpha) : Link(d, theta, r, alpha) { }
        void Move(float amount) { theta += amount;}
        float position(){ return d; };
        void position(float angle){ d = angle;};
};    