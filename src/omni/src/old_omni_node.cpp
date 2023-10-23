/**
**	ROS node for the Omni Geomagic Touch Haptic Device
**/

#include <ros/ros.h>
#include "chai3d.h"
#include <GLFW/glfw3.h>
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <iostream>
#include <sstream>

using namespace std;
using namespace chai3d;

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;

//------------------------------------------------------------------------------
// DECLARED VARIABLES
//------------------------------------------------------------------------------

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

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);


// this function closes the application
void close(void);

int main(int argc, char **argv){

    ///////////////////////////////////
    // NODE INITIALIZATION
    ///////////////////////////////////

    ros::init(argc,argv,"omni_node");
    // Access to communcations with ROS system
    ros::NodeHandle n;

    cout << endl;
    cout << "-----------------------------------" << endl;
    cout << "CHAI3D" << endl;
    cout << "Demo: Omni-KUKA pairng" << endl;
    cout << "Copyright 2003-2016" << endl;
    cout << "-----------------------------------" << endl << endl << endl;
    cout << "Keyboard Options:" << endl << endl;
    cout << "[1] - Wireframe (ON/OFF)" << endl;
    cout << "[2] - Increase Opacity" << endl;
    cout << "[3] - Reduce Opacity" << endl;
    cout << "[f] - Enable/Disable full screen mode" << endl;
    cout << "[m] - Enable/Disable vertical mirroring" << endl;
    cout << "[q] - Exit application" << endl;
    cout << endl << endl;

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
    // WORLD - CAMERA - LIGHTING
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

    //--------------------------------------------------------------------------
    // HAPTIC DEVICES / TOOLS
    //--------------------------------------------------------------------------
    
    // create a haptic device handler
    handler = new cHapticDeviceHandler();

    // get access to the first available haptic device found
    handler->getDevice(hapticDevice, 0);

    // retrieve information about the current haptic device
    cHapticDeviceInfo hapticDeviceInfo = hapticDevice->getSpecifications();

    // create a tool (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);

    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);

//    // if the haptic device has a gripper, enable it as a user switch
//    hapticDevice->setEnableGripperUserSwitch(true);

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
    double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;

    /////////////////////////////////////////////////////////////////////////
    // OBJECT
    /////////////////////////////////////////////////////////////////////////

    // create a sphere and define its radius
    object = new cShapeSphere(0.3);

    // add object to world
    world->addChild(object);

    // set the position of the object at the center of the world
    object->setLocalPos(0.0, 0.0, 0.0);

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
    object->m_texture->loadFromFile("resources/images/spheremap-3.jpg");

    // set graphic properties
    object->m_texture->setSphericalMappingEnabled(true);
    object->setUseTexture(true);
    object->m_material->setWhite();

    // set haptic properties
    object->m_material->setStiffness(0.6 * maxStiffness);          // % of maximum linear stiffness
    object->m_material->setViscosity(0.1 * maxDamping);            // % of maximum linear damping

    //------------------------------------------------------------------------------------
    //    Once the haptic values are defined in the material class, we still need to enable
    //    the effects that will use these values. In this example, we enable three haptic
    //    effects that occur when the tool touches the sphere.
    //    If the effects are not initialized, then the haptic values assigned to the
    //    material class are simply ignored.
    //------------------------------------------------------------------------------------

    // create a haptic surface effect
    object->createEffectSurface();
    // create a haptic viscous effect
    object->createEffectViscosity();

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
    drill->deleteCollisionDetector(true);

    // define a material property for the mesh
    cMaterial mat;
    mat.m_ambient.set(0.5f, 0.5f, 0.5f);
    mat.m_diffuse.set(0.8f, 0.8f, 0.8f);
    mat.m_specular.set(1.0f, 1.0f, 1.0f);
    drill->setMaterial(mat, true);
    drill->computeAllNormals();

    // attach drill to tool
    tool->m_image->addChild(drill);

    //--------------------------------------------------------------------------
    // SIDE FRAMEBUFFER
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

    //--------------------------------------------------------------------------
    // START SIMULATION
    //--------------------------------------------------------------------------
    
    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);
    
    // setup callback when application exits
    atexit(close);

    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window) || ros::ok())
    {
        
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        cout<<"Here!"<<endl;

        // render graphics
        updateGraphics();

        cout<<"Here too!"<<endl;

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

        // signal frequency counter
        freqCounterGraphics.signal(1);

        ////////////////////////////////////
        // MESSAGE PUBLICATION
        ////////////////////////////////////        

        ros::Publisher pub = n.advertise <std_msgs::String>("chatter", 1000);
        ros::Rate loop_rate(10);        

        int count = 0;
        cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();
        cMatrix3d rot = tool->getDeviceGlobalRot();

        cout<<pos.eigen()<<endl;
        cout<<rot.eigen()<<endl;

        std_msgs::String msg;
        std::stringstream ss;
        ss<<"hello world!"<<count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
        ++count;
        

    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

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

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
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

    cout<<"Here 2"<<endl;

    // update haptic and graphic rate data
    //labelRates->setText(cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / " +
        //cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

    // update position of label
    //labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

    cout<<"Here 2"<<endl;

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
    while(simulationRunning)
    {
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // signal frequency counter
        freqCounterHaptics.signal(1);

        // compute global reference frames for each object
        world->computeGlobalPositions(true);

        // update position and orientation of tool
        tool->updateFromDevice();

        // compute interaction forces
        tool->computeInteractionForces();

        // send forces to haptic device
        tool->applyToDevice();


        /////////////////////////////////////////////////////////////////////////
        // MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // if the haptic device does track orientations, we automatically
        // orient the drill to remain perpendicular to the object
        cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();
        cMatrix3d rot = tool->getDeviceGlobalRot();

        if (info.m_sensedRotation == false)
        {
            rot.identity();

            cVector3d vx, vy, vz;
            cVector3d zUp (0,0,1);
            cVector3d yUp (0,1,0);
            vx = pos - object->getLocalPos();
            if (vx.length() > 0.001)
            {
                vx.normalize();

                if (cAngle(vx,zUp) > 0.001)
                {
                    vy = cCross(zUp, vx);
                    vy.normalize();
                    vz = cCross(vx, vy);
                    vz.normalize();

                }
                else
                {
                    vy = cCross(yUp, vx);
                    vy.normalize();
                    vz = cCross(vx, vy);
                    vz.normalize();
                }

                rot.setCol(vx, vy, vz);
                drill->setLocalRot(rot);
            }
        }

        int button = tool->getUserSwitch(0);
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
    }

    // exit haptics thread
    simulationFinished = true;
}

//------------------------------------------------------------------------------
