
#include "omni_device.h"
#include "timer.h"
#include <thread>
#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <GLFW/glfw3.h>
#include </home/matteo/chai3d-3.2.0/src/devices/CDeltaDevices.h>
#include </home/matteo/chai3d-3.2.0/src/devices/CGenericHapticDevice.h>

#include <iostream>
using namespace std;

#include <chrono>
using namespace std::chrono;

State state;

//------------------------------------------------------------------------------
// DECLARED VARIABLES FOR Graphics Implementation
//------------------------------------------------------------------------------
// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;
bool insidews = true;
// fullscreen mode
bool fullscreen = false;
// mirrored display
bool mirroredDisplay = false;
// a world that contains all objects of the virtual environment
cWorld* world;
// a camera to render the world in the window display
cCamera* camera;
// a light source to illuminate the objects in the world
cDirectionalLight *light;
// virtual drill mesh
cMultiMesh* drill;
// virtual sphere
cShapeSphere* object;
cShapeSphere* object1;
// a haptic device handler
cHapticDeviceHandler* handler;
// a pointer to the current haptic device
cGenericHapticDevicePtr hapticDevice;
// a virtual tool representing the haptic device in the scene
cToolCursor* tool;
// side framebuffer
cFrameBufferPtr frameBuffer;
// a colored background
cBackground* background;
// side Panel that displays content of framebuffer
cViewPanel* viewPanel;
// angular velocity of object
cVector3d angVel(0.0, 0.0, 0.0);
// a font for rendering text
cFontPtr font;
// a label to display the rate [Hz] at which the simulation is running
cLabel* labelRates;
// level widget to display interaction forces
cLevel* level;
// dial widget to display velocity of torus
cDial* dial;
// a flag that indicates if the haptic simulation is currently running
bool simulationRunning = false;
// a flag that indicates if the haptic simulation has terminated
bool simulationFinished = true;
// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;
// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;
// haptic thread
cThread* hapticsThread;
// a handle to window display context
GLFWwindow* window = NULL;
// current width of window
int width = 0;
// current height of window
int height = 0;
// swap interval for the display context (vertical synchronization)
int swapInterval = 1;
// root resource path
string resourceRoot;
// Kuka pose data
double kuka_x,kuka_y,kuka_z,kuka_qx,kuka_qy,kuka_qz,kuka_qw;

//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------

// convert to resource path
string RESOURCE_PATH = "/home/matteo/chai3d-3.2.0/bin";

//------------------------------------------------------------------------------
// DECLARED FUNCTIONS
//------------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);
// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);
// this function renders the scene
void updateGraphics(void);
// this function contains the main haptics simulation loop
void updateHaptics(void);
// this function closes the application
void close(void);
// callback when an error GLFW occurs
void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}
// Callback to read the position of the end effector of the Kuka and compute the haptic force
void KukaPosCallback(const geometry_msgs::Pose &msg){
    kuka_x = msg.position.x;
    kuka_y = msg.position.y;
    kuka_z = msg.position.z;
    kuka_qx = msg.orientation.x;
    kuka_qy = msg.orientation.y;
    kuka_qz = msg.orientation.z;
    kuka_qw = msg.orientation.w;
}

// Main graphic loop
void ChaiGraphicLoop(void);


int main(int argc, char **argv){
    
    // Initialise haptic device
    omni_device omni;
    if(!omni.start()) return 0;
    handler = omni.get_handler();
    hapticDevice = omni.get_haptic_device();
   
     // Initialise kuka ROS node
    ros::init(argc,argv, "omni_node");
    ros::NodeHandle nh;
    ros::Publisher pub= nh.advertise<geometry_msgs::Pose>("Haptic_Dev/pose", 10000);
    ros::Subscriber sub = nh.subscribe("kuka_position", 1000, &KukaPosCallback);
    ros::Rate loop_rate(1000);

    auto me = getuid();
    auto myprivs = geteuid();

    cout<<"User IDs:"<<endl;
    cout<<me<<endl;
    cout<<myprivs<<endl;

    //--------------------------------------------------------------------------
    // OPEN GL - WINDOW DISPLAY
    //--------------------------------------------------------------------------

    // initialize GLFW library
    if (!glfwInit())
    {
        cout << "failed initialization" << endl;
        cSleepMs(1000);
        return 1;
    }

    // set error callback
    glfwSetErrorCallback(errorCallback);
    // compute desired size of window
    const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
    int w = 1 * mode->height;
    int h = 0.8 * mode->height;
    int x = 0.5 * (mode->width - w);
    int y = 0.5 * (mode->height - h);
    // set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
    // set active stereo mode
    if (stereoMode == C_STEREO_ACTIVE)
    {
        glfwWindowHint(GLFW_STEREO, GL_TRUE);
    }
    else
    {
        glfwWindowHint(GLFW_STEREO, GL_FALSE);
    }

    // create display context
    window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
    if (!window)
    {
        cout << "failed to create window" << endl;
        cSleepMs(1000);
        glfwTerminate();
        return 1;
    }

    // get width and height of window
    glfwGetWindowSize(window, &width, &height);
    // set position of window
    glfwSetWindowPos(window, x, y);
    // set key callback
    glfwSetKeyCallback(window, keyCallback);
    // set resize callback
    glfwSetWindowSizeCallback(window, windowSizeCallback);
    // set current display context
    glfwMakeContextCurrent(window);
    // sets the swap interval for the current display context
    glfwSwapInterval(swapInterval);

    #ifdef GLEW_VERSION
    // initialize GLEW library
    if (glewInit() != GLEW_OK)
    {
        cout << "failed to initialize GLEW library" << endl;
        glfwTerminate();
        return 1;
    }
    #endif

    //--------------------------------------------------------------------------
    // WORLD - CAMERA - LIGHTING - TOOL
    //--------------------------------------------------------------------------

    // create a new world.
    world = new cWorld();
    // set the background color of the environment
    world->m_backgroundColor.setBlack();

    // create a camera and insert it into the virtual world
    camera = new cCamera(world);

    world->addChild(camera);
    // position and orient the camera
    camera->set(cVector3d(2.5, 0.0, 0.6),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector

    // set the near and far clipping planes of the camera
    // anything in front or behind these clipping planes will not be rendered
    camera->setClippingPlanes(0.01, 10.0);
    // set stereo mode
    camera->setStereoMode(stereoMode);
    // set stereo eye separation and focal length (applies only if stereo is enabled)
    camera->setStereoEyeSeparation(0.03);
    camera->setStereoFocalLength(3.0);
    // set vertical mirrored display mode
    camera->setMirrorVertical(mirroredDisplay);

    // create a light source
    light = new cDirectionalLight(world);
    // attach light to camera
    camera->addChild(light);
    // enable light source
    light->setEnabled(true);
    // position the light source
    light->setLocalPos( 0.0, 0.5, 0.0);
    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);
    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);
    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

    //if the haptic device has a gripper, enable it as a user switch
    //hapticDevice->setEnableGripperUserSwitch(true);

    // define the radius of the tool (sphere)
    double toolRadius = 0.01;
    // define a radius for the tool
    tool->setRadius(toolRadius);
    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);
    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();
    // enable if objects in the scene are going to rotate of translate
    // or possibly collide against the tool. If the environment
    // is entirely static, you can set this parameter to "false"
    tool->enableDynamicObjects(false);
    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);
    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when
    // the tool is located inside an object for instance.
    tool->setWaitForSmallForce(true);
    // start the haptic tool
    tool->start();

    //--------------------------------------------------------------------------
    // COMPOSE OBJECTS
    //--------------------------------------------------------------------------

    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    double workspaceScaleFactor = tool->getWorkspaceScaleFactor();

    // get properties of haptic device
    cHapticDeviceInfo hapticDeviceInfo = omni.get_haptic_device_info();
    double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;

    // map physical workspace of haptic device to a simulation workspace.
    tool->setWorkspaceRadius(0.8);
    /////////////////////////////////////////////////////////////////////////
    // OBJECT
    /////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object = new cShapeSphere(0.4);
    // add object to world
    world->addChild(object);
    // set the position of the object at the center of the world
    object->setLocalPos(-0.4, 0.0, 0.0);

 //    // load an object file
//    bool fileload;
//    fileload = object->loadFromFile(RESOURCE_PATH("../resources/models/object/object.obj"));
//    if (!fileload)
//    {
//        #if defined(_MSVC)
//        fileload = object->loadFromFile("../../../bin/resources/models/object/object.obj");
//        #endif
//    }
//    if (!fileload)
//    {
//        printf("Error - 3D Model failed to load correctly.\n");
//        close();
//        return (-1);
//    }

    // compute a boundary box
    object->computeBoundaryBox(true);
    // create texture map
    object->m_texture = cTexture2d::create();
    // load texture map from file
    object->m_texture->loadFromFile("resources/images/stone-normal.png");
    // set graphic properties
    object->m_texture->setSphericalMappingEnabled(true);
    object->setUseTexture(true);
    object->m_material->setWhite();
    // set haptic properties
    object->m_material->setStiffness(15 * maxStiffness);          // % of maximum linear stiffness
    
    // create a haptic surface effect
    object->createEffectSurface();

    // // create a haptic magnetic effect
    // object0->createEffectMagnetic();
    // // create a haptic viscous effect
    // object0->createEffectViscosity();

    ////////////////////////////////////////////////////////////////////////
    // OBJECT 1: "FLUID"
    ////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object1 = new cShapeSphere(0.75);

    // add object to world
    world->addChild(object1);

    // set the position of the object at the center of the world
    object1->setLocalPos(-0.4, 0.0, 0.0);

    // load texture map
    object1->m_texture = cTexture2d::create();
    object1->m_texture->loadFromFile("resources/images/spheremap-2.jpg");

    // set graphic properties
    object1->m_material->m_ambient.set(0.1, 0.1, 0.6, 0.5);
    object1->m_material->m_diffuse.set(0.3, 0.3, 0.9, 0.5);
    object1->m_material->m_specular.set(1.0, 1.0, 1.0, 0.5);
    object1->m_material->setWhite();
    object1->setTransparencyLevel(0.01);
    object1->setUseTexture(true);
    object1->m_texture->setSphericalMappingEnabled(true);

    // set haptic properties
    object1->m_material->setViscosity(2 * maxDamping);    // % of maximum linear damping

    // create a haptic viscous effect
    object1->createEffectViscosity();

    /////////////////////////////////////////////////////////////////////////
    // OBJECT "DRILL"
    /////////////////////////////////////////////////////////////////////////

    // create a new mesh.
    drill = new cMultiMesh();

    // load a drill like mesh and attach it to the tool
    bool fileload;
    fileload = drill->loadFromFile(RESOURCE_PATH + "/resources/models/drill/drill.3ds");
    if (!fileload)
    {
        #if defined(_MSVC)
        fileload = drill->loadFromFile(RESOURCE_PATH("/resources/models/drill/drill.3ds"));
        #endif
    }
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        close();
        return (-1);
    }

    // resize tool mesh model
    drill->scale(0.004);

    // remove the collision detector. we do not want to compute any
    // force feedback rendering on the object itself.
    drill->deleteCollisionDetector(false);

    // define a material property for the mesh
    cMaterial mat;
    mat.m_ambient.set(0.5f, 0.5f, 0.5f);
    mat.m_diffuse.set(0.8f, 0.8f, 0.8f);
    mat.m_specular.set(1.0f, 1.0f, 1.0f);
    drill->setMaterial(mat, true);
    drill->computeAllNormals();

    // attach drill to tool
    //tool->m_image->addChild(drill);
    world->addChild(drill);

    //--------------------------------------------------------------------------
    // SIDE FRAMEBUFFER - CRATE A SECONDARY CAMERA THAT SHOW WHAT HAPPEN ON THE DRILL TIP
    //--------------------------------------------------------------------------

    // create secondary camera for side view
    cCamera* cameraTool = new cCamera(world);

    // attach camera to tool
    drill->addChild(cameraTool);
    cameraTool->setLocalPos(0.0, 0.0, 0.1);

    // create framebuffer for side view
    frameBuffer = cFrameBuffer::create();
    frameBuffer->setup(cameraTool);

    // create panel to display side view
    viewPanel = new cViewPanel(frameBuffer);
    camera->m_frontLayer->addChild(viewPanel);
    viewPanel->setLocalPos(10, 10);

    bool b1, b2;
    b1=b2=false;
    double home_x =0;
    double home_y =0;
    double home_z =0;
    double fx, fy, fz;  
    double beta = 80; //Damping coefficient
    double viscous = 20000; //Reduce the velocity when I'm close to the object
    
    double homing_K = 500;
    double main_K = 300;
    double k_scale=1.0;
    double homing_tol = 0.005;
    double homing_T =10.0;
    cVector3d home_pos(0.00,0.00,0.00);

    state = homing;
    high_resolution_clock::time_point last_state_change = high_resolution_clock::now();

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
    
    // setup callback when application exits
    atexit(close);

    // while(state == homing){
    //     double x, y, z;
    //     double qx,qy,qz,qw;
    //     omni.get_position(x,y,z);
    //     omni.get_orientation(qx,qy,qz,qw); 
    //     fx=fy=fz=0.0;
    //     k_scale=1.0;

    //     if(std::chrono::duration_cast<std::chrono::seconds>(high_resolution_clock::now() - last_state_change).count()<homing_T){
    //         auto dt = std::chrono::duration_cast<std::chrono::seconds>(high_resolution_clock::now() - last_state_change);
    //         k_scale = dt.count()/homing_T;               
    //            k_scale = dt.count()/homing_T;               
    //         k_scale = dt.count()/homing_T;               
    //     }
    //     fx=k_scale*homing_K*(home_x-x);
    //     fy=k_scale*homing_K*(home_y-y);
    //     fz=k_scale*homing_K*(home_z-z);

    //     if(b1){
    //         high_resolution_clock::time_point t = high_resolution_clock::now();
    //         auto dt = std::chrono::duration_cast<std::chrono::seconds>(t - last_state_change);
    //         if(dt.count()>1.0){
    //             double err_x=x-home_x;
    //             double err_y=y-home_y;
    //             double err_z=z-home_z;
    //             if(sqrt(err_x*err_x+err_y*err_y+err_z*err_z)<homing_tol){
    //                 state=main_loop;
    //                 last_state_change = high_resolution_clock::now();                   
    //                 last_state_change = high_resolution_clock::now();                   
    //                 last_state_change = high_resolution_clock::now();                   
    //                 cout<<"Entering main loop"<<endl;
    //             }
    //         }
    //     }
    //     omni.set_force(fx,fy,fz);        
    //     omni.get_button_state(b1,b2);
    // }

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------

    // omni.set_reference_position(0, 0, 0);

    // if(!omni.enable_force()){
    //     cout<<"Unable to activate the Haptic control."<<endl;
    //     return 0;
    // }

    vector<double> kukaPrevPos{0.0,0.0,0.0};
    vector<double> kukaActualPos{0.0,0.0,0.0};
    
    mimlab::Timer timer;
    while (state !=exiting && ros::ok()){
        double x, y, z;
        double qx,qy,qz,qw;
        geometry_msgs::Pose pos;
        omni.get_position(x,y,z);
        omni.get_orientation(qw,qx,qy,qz);
        fx=fy=fz=0.0;
        cVector3d speed(0.0, 0.0, 0.0);

        // kukaActualPos = {kuka_x,kuka_y,kuka_z};

        // if (kukaActualPos != kukaPrevPos){
        //         insidews = false;
        //     }
        
        // else{
        //     insidews = true;
        // }

        hapticDevice->getLinearVelocity(speed);

        switch(state){
            
            case homing:

            k_scale=1.0;

            if(std::chrono::duration_cast<std::chrono::seconds>(high_resolution_clock::now() - last_state_change).count()<homing_T){
                auto dt = std::chrono::duration_cast<std::chrono::seconds>(high_resolution_clock::now() - last_state_change);
                k_scale = dt.count()/homing_T;               
                k_scale = dt.count()/homing_T;               
                k_scale = dt.count()/homing_T;               
            }
            fx=k_scale*homing_K*(home_x-x)- beta*speed(1)/1000;
            fy=k_scale*homing_K*(home_y-y)- beta*speed(2)/1000;
            fz=k_scale*homing_K*(home_z-z)- beta*speed(3)/1000;

            pos.position.x = x;
            pos.position.y = y;
            pos.position.z = z;
            pos.orientation.w = qw;
            pos.orientation.x = qx;
            pos.orientation.y = qy;
            pos.orientation.z = qz;
            
            pub.publish(pos);
            ros::spinOnce();

            if(b1){

                high_resolution_clock::time_point t = high_resolution_clock::now();
                auto dt = std::chrono::duration_cast<std::chrono::seconds>(t - last_state_change);
                if(dt.count()>1.0){
                    double err_x=x-home_x;
                    double err_y=y-home_y;
                    double err_z=z-home_z;
                    if(sqrt(err_x*err_x+err_y*err_y+err_z*err_z)<homing_tol){
                        state=main_loop;
                        last_state_change = high_resolution_clock::now();                   
                        last_state_change = high_resolution_clock::now();                   
                        last_state_change = high_resolution_clock::now();                   
                        cout<<"Entering main loop"<<endl;
                    }
                }
            }
            omni.set_force(fx,fy,fz,state);        
            break;

            case main_loop:
            
            ChaiGraphicLoop();
            timer.tick();
            
            pos.position.x = x;
            pos.position.y = y;
            pos.position.z = z;
            pos.orientation.w = qw;
            pos.orientation.x = qx;
            pos.orientation.y = qy;
            pos.orientation.z = qz;
            
            pub.publish(pos);
            ros::spinOnce();

            cout<<x<<' '<<y<<' '<<z<<' '<<endl;

            if(insidews == false){

            //     //omni.set_reference_position(kuka_x,kuka_y,kuka_z);
            //     fx=k_scale*main_K*(kuka_x /*-x*/) - beta*speed(1)/1000;
            //     fy=k_scale*main_K*(kuka_y/*-y*/) - beta*speed(2)/1000;
            //     fz=k_scale*main_K*(kuka_z/*-z*/) - beta*speed(3)/1000;

            //     omni.set_force(fx,fy,fz,state);
            }

            // if(0.0<abs(x)<0.021 || 0.0<abs(y)<0.021 || 0.0<abs(z)<0.021){
            //     fx =  - viscous*speed(1);
            //     fy =  - viscous*speed(2);
            //     fz =  - viscous*speed(3);

            //     omni.set_force(fx,fy,fz,state);
            // }

            // kukaPrevPos = kukaActualPos;

            if(b1){
                high_resolution_clock::time_point t = high_resolution_clock::now();
                auto dt = std::chrono::duration_cast<std::chrono::seconds>(t - last_state_change);
                if(dt.count()>1.0){ //Check that at least 1 second has passed
                    state=exiting;
                    last_state_change = high_resolution_clock::now();                   
                    cout<<"Exiting main loop"<<endl;
                }
            }
            
            break;          
        }

        omni.get_button_state(b1,b2);

        loop_rate.sleep();
    }    

    cout<<timer.get_stats()<<endl;
    // if(!omni.disable_force()){
    //     cout<<"Unable to deactivate the Haptic control."<<endl;
    //     return 0;
    // }
    // Exiting program
    // ----------------------------------------------------------------------------
    // close window
    glfwDestroyWindow(window);
    // terminate GLFW library
    glfwTerminate();
    omni.set_force(0,0,0,state);
    omni.stop();

    return 0;
}


// FUNCTION DECLARATION ------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
    // update size of framebuffer and view panel
    int side = 0.3 * cMin(width, height);
    frameBuffer->setSize(side, side);
    
    int radius = 0.25 * side;
    viewPanel->setSize(side,side);
    viewPanel->setCornerRadius(radius, radius, radius, radius);
}

//------------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
    // filter calls that only include a key press
    if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
    {
        return;
    }

    // option - exit
    else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
    {
        glfwSetWindowShouldClose(a_window, GLFW_TRUE);
    }

    // option - toggle fullscreen
    else if (a_key == GLFW_KEY_F)
    {
        // toggle state variable
        fullscreen = !fullscreen;

        // get handle to monitor
        GLFWmonitor* monitor = glfwGetPrimaryMonitor();

        // get information about monitor
        const GLFWvidmode* mode = glfwGetVideoMode(monitor);

        // set fullscreen or window mode
        if (fullscreen)
        {
            glfwSetWindowMonitor(window, monitor, 0, 0, mode->width, mode->height, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
        else
        {
            int w = 0.8 * mode->height;
            int h = 0.5 * mode->height;
            int x = 0.5 * (mode->width - w);
            int y = 0.5 * (mode->height - h);
            glfwSetWindowMonitor(window, NULL, x, y, w, h, mode->refreshRate);
            glfwSwapInterval(swapInterval);
        }
    }

    // option - toggle vertical mirroring
    else if (a_key == GLFW_KEY_M)
    {
        mirroredDisplay = !mirroredDisplay;
        camera->setMirrorVertical(mirroredDisplay);
    }
}

//------------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    tool->stop();

    // delete resources
    delete hapticsThread;
    delete world;
    delete handler;
}

//------------------------------------------------------------------------------

void updateGraphics(void)
{
    /////////////////////////////////////////////////////////////////////
    // UPDATE WIDGETS
    /////////////////////////////////////////////////////////////////////

    // update haptic and graphic rate data
    //labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        //cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    //labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    /////////////////////////////////////////////////////////////////////
    // RENDER SCENE
    /////////////////////////////////////////////////////////////////////

    // update shadow maps (if any)
    world->updateShadowMaps(false, mirroredDisplay);

    // render side framebuffer
    frameBuffer->renderView();

    // render world
    camera->renderView(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}

//------------------------------------------------------------------------------

void updateHaptics(void)
{
    // temp variable to store positions and orientations
    // of object and drill
    cVector3d lastPosObject;
    cMatrix3d lastRotObject;
    cVector3d lastPosDevice;
    cMatrix3d lastRotDevice;
    cVector3d lastDeviceObjectPos;
    cMatrix3d lastDeviceObjectRot;
    bool firstTime = false;

    // retrieve information about the current haptic device
    cHapticDeviceInfo info = hapticDevice->getSpecifications();

    // simulation in now running
    simulationRunning  = true;
    simulationFinished = false;

    // main haptic simulation loop
    mimlab::Timer timer;
    
    while(simulationRunning)
    {
    
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        if(state==main_loop && insidews == true){
            
            //update position and orientation of tool
            tool->updateFromDevice();

            //compute interaction forces
            tool->computeInteractionForces();

            //send forces to haptic device
            tool->applyToDevice();        
        }

        /////////////////////////////////////////////////////////////////////////
        // MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // if the haptic device does track orientations, we automatically
        // orient the drill to remain perpendicular to the object
        // cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();
        cVector3d pos; 	    
        hapticDevice->getPosition(pos);
        pos*=tool->getWorkspaceScaleFactor();
        cMatrix3d rot = tool->getDeviceGlobalRot();

        // if (info.m_sensedRotation == false)
        // {
        //     rot.identity();

        //     cVector3d vx, vy, vz;
        //     cVector3d zUp (0,0,1);
        //     cVector3d yUp (0,1,0);
        //     vx = pos - object->getLocalPos();
        //     if (vx.length() > 0.001)
        //     {
        //         vx.normalize();

        //         if (cAngle(vx,zUp) > 0.001)
        //         {
        //             vy = cCross(zUp, vx);
        //             vy.normalize();
        //             vz = cCross(vx, vy);
        //             vz.normalize();

        //         }
        //         else
        //         {
        //             vy = cCross(yUp, vx);
        //             vy.normalize();
        //             vz = cCross(vx, vy);
        //             vz.normalize();
        //         }

        //         rot.setCol(vx, vy, vz);
        //         drill->setLocalRot(rot);
        //     }
        // }


        // rot.identity();

        // cVector3d vx, vy, vz;
        // cVector3d zUp (0,0,1);
        // cVector3d yUp (0,1,0);
        // vx = pos - object->getLocalPos();
        // if (vx.length() > 0.001)
        // {
        //     vx.normalize();

        //     if (cAngle(vx,zUp) > 0.001)
        //     {
        //         vy = cCross(zUp, vx);
        //         vy.normalize();
        //         vz = cCross(vx, vy);
        //         vz.normalize();

        //     }
        //     else
        //     {
        //         vy = cCross(yUp, vx);
        //         vy.normalize();
        //         vz = cCross(vx, vy);
        //         vz.normalize();
        //     }

        //     rot.setCol(vx, vy, vz);
        //     drill->setLocalRot(rot);
        // }
            
        
        drill->setLocalRot(rot);
        drill->setLocalPos(pos);


        int button = tool->getUserSwitch(2);
        if (button == 0)
        {
            lastPosDevice = pos;
            lastRotDevice = rot;
            lastPosObject = object->getLocalPos();
            lastRotObject = object->getLocalRot();
            lastDeviceObjectPos = cTranspose(lastRotDevice) * ((lastPosObject - lastPosDevice) + 0.01*cNormalize(lastPosObject - lastPosDevice));
            lastDeviceObjectRot = cMul(cTranspose(lastRotDevice), lastRotObject);
            drill->setTransparencyLevel (1.0, true, true);
            firstTime = true;
        }
        else if (firstTime)
        {
            drill->setTransparencyLevel (0.3, true, true);
            cMatrix3d newRot =  cMul(rot, lastDeviceObjectRot);
            cVector3d newPos = cAdd(pos, cMul(rot, lastDeviceObjectPos));
            object->setLocalPos(newPos);
            object->setLocalRot(newRot);
            world->computeGlobalPositions(true);
        }
        timer.tick();
    }

    // exit haptics thread
    simulationFinished = true;
    cout<<"Haptic cicle speed: "<<timer.get_stats()<<endl;
}

//------------------------------------------------------------------------------

void ChaiGraphicLoop(void)
{
        // get width and height of window
        //glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);

        // create a thread which starts the main haptics rendering loop

}
