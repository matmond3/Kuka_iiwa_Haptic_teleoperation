#pragma Once

#include "chai3d.h"
#include <mutex>
#include <thread>

using namespace chai3d;
using namespace std;

typedef enum {
    homing,
    main_loop,
    exiting
}State;

class omni_device{
    public:
    omni_device();
    ~omni_device();
    bool start();
    bool stop();
    bool get_button_state(bool & button1_state, bool & button2_state);
    bool get_position(double &x, double &y, double &z);
    bool get_orientation(double &qw, double &qx, double &qy,double &qz);
    bool set_reference_position(double x, double y, double z);
    bool set_force(double fx, double fy, double fz, State state);
    bool enable_force();
    bool disable_force();
    bool has_actuated_rotation();
    cGenericHapticDevicePtr get_haptic_device();
    cHapticDeviceInfo get_haptic_device_info();
    cHapticDeviceHandler* get_handler();

    friend void updateForces(omni_device * omni);    
    
    private:

    bool haptic_thread_running;
    double refX, refY, refZ;

    cHapticDeviceHandler* handler;
    
    // a pointer to the current haptic device
    cGenericHapticDevicePtr hapticDevice;   

    mutex mtx;
    std::thread force_thread;

};
