#ifndef _FAKE_MAG_H_
#define _FAKE_MAG_H_


//This is a pseudo Magnetometer Class as a placeholder for IMUs that do not have a Magnetometer

class FakeMag
{
    public:
        FakeMag();
        bool initialize();
        bool testConnection();
        void getHeading(int16_t* mx, int16_t* my, int16_t* mz);
};

FakeMag::FakeMag()
{

}

bool  FakeMag::initialize()
{
    return true;
}

bool  FakeMag::testConnection()
{
    return true;
}

void FakeMag::getHeading(int16_t* mx, int16_t* my, int16_t* mz)
{
    *mx = 0;
    *my = 0;
    *mz = 0;
}

#endif