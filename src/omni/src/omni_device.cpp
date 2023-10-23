#include "omni_device.h"
#include "timer.h"
#include <iostream>
using namespace std;

omni_device::omni_device(){
    hapticDevice=NULL;    
    // create a haptic device handler
    handler = new cHapticDeviceHandler();
}

omni_device::~omni_device(){
    delete handler;
}

bool omni_device::start(){    

    if(handler->getNumDevices()==0){
        cout<<"Error! No haptic devices connected"<<endl;
        return false;
    }

    // get access to the first available haptic device found
    if(handler->getDevice(hapticDevice, 0)){
        cout<<"Connection..."<<endl;
        
    }

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();
    cout<< "Found haptic device: "<<hapticDeviceInfo.m_manufacturerName<<" "<<hapticDeviceInfo.m_modelName<<endl;


    if(hapticDevice->open()){
        cout<<"Open connection to haptic device"<<endl;
        
    }

    // calibrate device (if necessary)
    if(!hapticDevice->calibrate()){
        cout<<"Error! Could not calibrate haptic device"<<endl;
        return false;
    }

    //hapticDevice->setEnableGripperUserSwitch (true);

    if(hapticDevice->isDeviceReady()){
        cout<<"Successfully connected to haptic device"<<endl;
        return true;        
    }else{
        return false;
    }
}

bool omni_device::stop(){
    if(hapticDevice==NULL) return true;
    
    hapticDevice->close();

    cout<<"Successfully disconnected haptic device"<<endl;

    return true;    
}

bool omni_device::get_button_state(bool & button1_state, bool & button2_state){
    if(hapticDevice==NULL) return false;    

    hapticDevice->getUserSwitch(0, button1_state);
    hapticDevice->getUserSwitch(1, button2_state);

    return true;
}

bool omni_device::get_position(double &x, double &y, double &z){
    if(hapticDevice==NULL) return false;    

    cVector3d position(0.0,0.0,0.0); 	    

    hapticDevice->getPosition(position);

    x=position.get(0);
    y=position.get(1);
    z=position.get(2);

    //std::cout<<"POS: "<<x<<" "<<y<<" "<<z<<endl;

    return true;
}

bool omni_device::get_orientation(double &qw, double &qx, double &qy,double &qz){
    if(hapticDevice==NULL) return false; 

    cMatrix3d rotation(0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0,0.0);

    hapticDevice->getRotation(rotation);

    cQuaternion quat;
    quat.fromRotMat(rotation);
    
    qx=quat.x;
    qy=quat.y;
    qz=quat.z;
    qw=quat.w;

    
    //std::cout<<"QUAT: "<<qx<<" "<<qy<<" "<<qz<<endl; 

    return true;
}

void updateForces(omni_device * omni){
    while(omni->haptic_thread_running){
        if(omni->hapticDevice==NULL) return;

        double x,y,z;
        omni->get_position(x, y, z);

        double fx, fy, fz;
        double homing_T =10.0;
        double k_scale=1.0;
        //double e_x =0.1;
        //double e_y =0.1;
        //double e_z =0.1;
        double homing_K = 300;

        fx=fy=fz=0.0;
        //double ref_x = omni->refX;
        //double ref_y = omni->refY;
        //double ref_z = omni->refZ;

        omni->mtx.lock();
        fx=homing_K*(omni->refX-x);
        fy=homing_K*(omni->refY-y);
        fz=homing_K*(omni->refZ-z);
        omni->mtx.unlock();

        double f_mag = sqrt(fx*fx+fy*fy+fz*fz);
        if(f_mag>omni->hapticDevice->getSpecifications().m_maxLinearForce){
            double scale =omni->hapticDevice->getSpecifications().m_maxLinearForce/f_mag;
            fx*=scale;
            fy*=scale;
            fz*=scale;
        }

        cVector3d force(fx, fy, fz);
        cout<<"Forces"<<endl;
        cout<<fx<<' '<<fy<<' '<<fz<<endl;
        if(!omni->hapticDevice->setForce(force)) return;
    }
}

bool omni_device::enable_force(){

    haptic_thread_running = true;
    force_thread=std::thread(updateForces,this);
    force_thread.detach();
    return true;
}

bool omni_device::disable_force(){

    haptic_thread_running = false;
    force_thread.join();

    double fx, fy, fz;
    fx=fy=fz=0.0;
    cVector3d force(fx, fy, fz);
    
    if(!hapticDevice->setForce(force)) return false;
    

    return true;
}

bool omni_device::has_actuated_rotation(){
    if(hapticDevice==NULL) return false;    

    return hapticDevice->getSpecifications().m_actuatedRotation;

}

cHapticDeviceInfo omni_device::get_haptic_device_info(){

    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    return hapticDeviceInfo;
}

cGenericHapticDevicePtr omni_device::get_haptic_device(){

    return hapticDevice; 
}

cHapticDeviceHandler* omni_device::get_handler(){
    
    return handler;
}

bool omni_device::set_reference_position(double x, double y, double z){
    mtx.lock();
    refX = x;
    refY = y;
    refZ = z;
    cout<<"REF POS: "<<refX<<' '<<refY<<' '<<refZ<<endl;
    mtx.unlock();
    return true;
}

// void omni_device::updateForces(){
//     while(haptic_thread_running){
//         if(hapticDevice==NULL) return;

//         double fx, fy, fz;
//         double homing_T =10.0;
//         double k_scale=1.0;
//         double e_x =0.1;
//         double e_y =0.1;
//         double e_z =0.1;
//         double homing_K = 500;

//         fx=fy=fz=0.0;
//         // double ref_x = omni->refX;
//         // double ref_y = omni->refY;
//         // double ref_z = omni->refZ;

//         fx=homing_K*(e_x-refX);
//         fy=homing_K*(e_y-refY);
//         fz=homing_K*(e_z-refZ);

//         double f_mag = sqrt(fx*fx+fy*fy+fz*fz);
//         if(f_mag>hapticDevice->getSpecifications().m_maxLinearForce){
//             double scale =hapticDevice->getSpecifications().m_maxLinearForce/f_mag;
//             fx*=scale;
//             fy*=scale;
//             fz*=scale;
//         }

//         cVector3d force(fx, fy, fz);
//         if(!hapticDevice->setForce(force)) return;
//     }
// }

bool omni_device::set_force(double fx, double fy, double fz, State state){
    if(hapticDevice==NULL) return false;
    double tx,ty,tz;
    double qx,qy,qz,qw;

    get_orientation(qw,qx,qy,qz);

    if(state == homing){
        tx= 0.2*(0.00-qx);
        ty= 1*(0.00-qy);
        tz= 0.1*(0.00-qz);
    }

    else{
        tx= 0.0;
        ty= 0.0;
        tz= 0.0;
    }
    double f_mag = sqrt(fx*fx+fy*fy+fz*fz);
    if(f_mag>hapticDevice->getSpecifications().m_maxLinearForce){
        double scale =hapticDevice->getSpecifications().m_maxLinearForce/f_mag;
        fx*=scale;
        fy*=scale;
        fz*=scale;
    }

    cVector3d force(fx, fy, fz);
    cVector3d torque(tx, ty, tz);
    if(!hapticDevice->setForceAndTorque(force, torque)) return false;
}