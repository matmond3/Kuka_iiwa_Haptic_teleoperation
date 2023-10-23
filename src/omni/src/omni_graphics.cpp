#include "chai3d.h"
#include <GLFW/glfw3.h>
#include "graphic_class.h"
#include "omni_device.h"
#include <iostream>

using namespace chai3d;
using namespace std;

string RESOURCE_PATH = "/home/matteo/chai3d-3.2.0/bin";

// this function renders the scene
void updateGraphics(void *void_GC);

//------------------------------------------------------------------------------

graphic_class::graphic_class(cGenericHapticDevicePtr hapticDevice, GLFWwindow* window){
    this->window=window;
    this->hapticDevice=hapticDevice;
    hapticDeviceInfo=hapticDevice->getSpecifications();

    simulationRunning = false;
    simulationFinished = true;    
    width = 0;
    height = 0;
    swapInterval = 1;
    mirroredDisplay = false;
    graphicsThread=NULL;
    world=NULL;    
}

graphic_class::~graphic_class(){
    if(world!=NULL)
        delete world;
}

bool graphic_class::environment(){    
    // create a new WORLD
    world = new cWorld();
    
    // set the background color of the environment
    world->m_backgroundColor.setBlack();
    
    // create a new CAMERA
    camera = new cCamera(world);
    world->addChild(camera);
    // position and orient the camera
    camera->set(cVector3d(2.5, 0.0, 0.6),    // camera position (eye)
                cVector3d(0.0, 0.0, 0.0),    // lookat position (target)
                cVector3d(0.0, 0.0, 1.0));   // direction of the (up) vector
    
    // set the near and far clipping planes of the camera
    camera->setClippingPlanes(0.01, 10.0);

    // create a new LIGHT
    light = new cDirectionalLight(world);
    camera->addChild(light);
    // enable light source
    light->setEnabled(true);
    // position the light source
    light->setLocalPos( 0.0, 0.5, 0.0);
    // define the direction of the light beam
    light->setDir(-3.0,-0.5, 0.0);    

    /*
    // create a TOOL (cursor) and insert into the world
    tool = new cToolCursor(world);
    world->addChild(tool);
    // connect the haptic device to the virtual tool
    tool->setHapticDevice(hapticDevice);
    // define the radius of the tool (sphere)
    double toolRadius = 0.01;
    tool->setRadius(toolRadius);
    // hide the device sphere. only show proxy.
    tool->setShowContactPoints(true, false);
    // create a white cursor
    tool->m_hapticPoint->m_sphereProxy->m_material->setWhite();
    // map the physical workspace of the haptic device to a larger virtual workspace.
    tool->setWorkspaceRadius(1.0);
    // haptic forces are enabled only if small forces are first sent to the device;
    // this mode avoids the force spike that occurs when the application starts when
    // the tool is located inside an object for instance.
    tool->setWaitForSmallForce(true);
    // start the haptic tool
    if(!tool->start()){
        cout<<"Error during the initialization of the haptic virtual tool"<<endl;
        return false;
    }
    
    // create a new mesh: DRILL
    drill = new cMultiMesh();
    bool fileload;
    fileload = drill->loadFromFile(RESOURCE_PATH + "/resources/models/drill/drill.3ds");
    if (!fileload)
    {
        printf("Error - 3D Model failed to load correctly.\n");
        stop();
        return false;
    }
    // resize tool mesh model
    drill->scale(0.004);
    // define a material property for the mesh
    cMaterial mat;
    mat.m_ambient.set(0.5f, 0.5f, 0.5f);
    mat.m_diffuse.set(0.8f, 0.8f, 0.8f);
    mat.m_specular.set(1.0f, 1.0f, 1.0f);
    drill->setMaterial(mat, true);
    drill->computeAllNormals();
    // attach drill to tool
    tool->m_image->addChild(drill);
    */

    return true;
}

bool graphic_class::def_object(){
    // read the scale factor between the physical workspace of the haptic
    // device and the virtual workspace defined for the tool
    //double workspaceScaleFactor = tool->getWorkspaceScaleFactor();
    // get properties of haptic device
    //double maxLinearForce = cMin(hapticDeviceInfo.m_maxLinearForce, 7.0);
    //double maxStiffness = hapticDeviceInfo.m_maxLinearStiffness / workspaceScaleFactor;
    //double maxDamping   = hapticDeviceInfo.m_maxLinearDamping / workspaceScaleFactor;

    // create a sphere and define its radius
    //object = new cShapeSphere(0.3);
    // add object to world
    //world->addChild(object);
    // set the position of the object at the center of the world
    //object->setLocalPos(0.0, 0.0, 0.0);
    // set haptic properties
    //object->m_material->setStiffness(0.6 * maxStiffness);          // % of maximum linear stiffness
    //object->m_material->setViscosity(0.1 * maxDamping);            // % of maximum linear damping
    // create a haptic surface effect
    //object->createEffectSurface();

    return true;
}

bool graphic_class::start(){    

    if(!environment()){
        cout<<"Could not generate CHAI3D environment"<<endl;
        return false;
    }
    if(!def_object()){
        cout<<"Could not generate CHAI3D object"<<endl;
        return false;
    }    

    // call window size callback at initialization    
    windowSizeCallback(window, width, height);
    
    graphicsThread = new cThread();
    graphicsThread->start(updateGraphics, CTHREAD_PRIORITY_GRAPHICS, (void *) this);
    
    /*
    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);
        // render graphics
        updateGraphics((void *) this);
        // swap buffers
        glfwSwapBuffers(window);
        // process events
        glfwPollEvents();      

        int count = 0;
        cVector3d pos = tool->m_hapticPoint->getGlobalPosProxy();
        cMatrix3d rot = tool->getDeviceGlobalRot();

    }
    */

    return true;
}

void graphic_class::windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
    
    // update size of framebuffer and view panel
    //int side = 0.3 * cMin(width, height);
    //frameBuffer->setSize(side, side);    

    //int radius = 0.25 * side;
    //viewPanel->setSize(side,side);
    //viewPanel->setCornerRadius(radius, radius, radius, radius);
}

//------------------------------------------------------------------------------

bool graphic_class::stop()
{
    // stop the simulation
    simulationRunning = false;

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // close haptic device
    //tool->stop();    

    // delete resources
    if(graphicsThread!=NULL)
        delete graphicsThread;

    if(world!=NULL)
        delete world;

    return true;
}

//------------------------------------------------------------------------------

void updateGraphics(void *void_GC)
{
    graphic_class * GC = (graphic_class *) void_GC;

    // get width and height of window
    glfwGetWindowSize(GC->window, &(GC->width), &(GC->height));

    // temp variable to store positions and orientations
    // of object and drill
    cVector3d lastPosObject;
    cMatrix3d lastRotObject;
    cVector3d lastPosDevice;
    cMatrix3d lastRotDevice;
    cVector3d lastDeviceObjectPos;
    cMatrix3d lastDeviceObjectRot;
    bool firstTime = false;    

    // simulation in now running
    GC->simulationRunning  = true;
    GC->simulationFinished = false;

    // main haptic simulation loop    
    while(GC->simulationRunning)
    {
        cout<<"Rendering Here"<<endl;

        /*
        /////////////////////////////////////////////////////////////////////////
        // HAPTIC RENDERING
        /////////////////////////////////////////////////////////////////////////

        // compute global reference frames for each object
        GC->world->computeGlobalPositions(true);

        // update position and orientation of tool
        //GC->tool->updateFromDevice();        

        // compute interaction forces
        //GC->tool->computeInteractionForces();

        // send forces to haptic device
        //GC->tool->applyToDevice();

        /////////////////////////////////////////////////////////////////////////
        // MANIPULATION
        /////////////////////////////////////////////////////////////////////////

        // if the haptic device does track orientations, we automatically
        // orient the drill to remain perpendicular to the object
        cVector3d pos(0,0,0); //= GC->tool->m_hapticPoint->getGlobalPosProxy();
        cMatrix3d rot(0,0,0); //= GC->tool->getDeviceGlobalRot();        

        if (GC->hapticDeviceInfo.m_sensedRotation == false)
        {
            rot.identity();

            cVector3d vx, vy, vz;
            cVector3d zUp (0,0,1);
            cVector3d yUp (0,1,0);
            vx = pos - GC->object->getLocalPos();
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
                GC->drill->setLocalRot(rot);
            }            
        }

        int button = 0; //GC->tool->getUserSwitch(0);
        if (button == 0)
        {
            lastPosDevice = pos;
            lastRotDevice = rot;
            lastPosObject = GC->object->getLocalPos();
            lastRotObject = GC->object->getLocalRot();
            lastDeviceObjectPos = cTranspose(lastRotDevice) * ((lastPosObject - lastPosDevice) + 0.01*cNormalize(lastPosObject - lastPosDevice));
            lastDeviceObjectRot = cMul(cTranspose(lastRotDevice), lastRotObject);
            GC->drill->setTransparencyLevel (1.0, true, true);
            firstTime = true;
        }
        else if (firstTime)
        {
            GC->drill->setTransparencyLevel (0.3, true, true);
            cMatrix3d newRot =  cMul(rot, lastDeviceObjectRot);
            cVector3d newPos = cAdd(pos, cMul(rot, lastDeviceObjectPos));
            GC->object->setLocalPos(newPos);
            GC->object->setLocalRot(newRot);
            GC->world->computeGlobalPositions(true);
        }        

        */

        /////////////////////////////////////////////////////////////////////
        // RENDER SCENE
        /////////////////////////////////////////////////////////////////////

        // update shadow maps (if any)
        //GC->world->updateShadowMaps(false, GC->mirroredDisplay);        

        // render side framebuffer
        //GC->frameBuffer->renderView();        

        // render world
        //GC->camera->renderView(GC->width, GC->height);
                

        // wait until all GL commands are completed
        //glFinish();        

        // swap buffers
        glfwSwapBuffers(GC->window);
        // process events
        glfwPollEvents();          

        cout<<"Rendering Here too"<<endl;                 
        
    }
    
    // exit haptics thread
    GC->simulationFinished = true;
}
