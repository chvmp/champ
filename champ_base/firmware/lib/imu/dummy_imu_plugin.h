#ifndef _DUMMY_IMU_PLUGIN_H_
#define _DUMMY_IMU_PLUGIN_H_

namespace DummyIMU
{   
    class Plugin
    {
        public:
            Plugin()
            {
                initialize();
            }

            void initialize()
            {
            }

            float readGyroscopeX()
            {
            }

            float readGyroscopeY()
            {
            }

            float readGyroscopeZ()
            {
            }

            float readAccelerometerX()
            {
            }

            float readAccelerometerY()
            {
            }

            float readAccelerometerZ()
            {
            }

            float readMagnetometerX()
            {
            }

            float readMagnetometerY()
            {
            }

            float readMagnetometerZ()
            {
            }
    };
}

#endif

