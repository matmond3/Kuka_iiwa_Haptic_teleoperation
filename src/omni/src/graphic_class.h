#pragma Once

#include "chai3d.h"
#include <GLFW/glfw3.h>
using namespace chai3d;

class graphic_class{

    public:
    
    graphic_class(cGenericHapticDevicePtr hapticDevice, GLFWwindow* window);
    ~graphic_class();
    
    bool start();    
    bool stop();

    friend void updateGraphics(void *void_GC);    

    private:

    bool environment();
    bool def_object();

    // callback when the window display is resized
    void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

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

    // a virtual tool representing the haptic device in the scene
    // cToolCursor* tool;

    // side framebuffer
    cFrameBufferPtr frameBuffer;

    // a colored background
    cBackground* background;

    // side Panel that displays content of framebuffer
    cViewPanel* viewPanel;

    // a font for rendering text
    cFontPtr font;

    // a label to display the rate [Hz] at which the simulation is running
    cLabel* labelRates;

    // level widget to display interaction forces
    cLevel* level;

    // a flag that indicates if the haptic simulation is currently running
    bool simulationRunning;

    // a flag that indicates if the haptic simulation has terminated
    bool simulationFinished;

    // graphic thread
    cThread* graphicsThread;

    // a handle to window display context
    GLFWwindow* window;

    // current width of window
    int width;

    // current height of window
    int height;

    // swap interval for the display context (vertical synchronization)
    int swapInterval;

    // mirrored display
    bool mirroredDisplay;

    cGenericHapticDevicePtr hapticDevice;   
    cHapticDeviceInfo hapticDeviceInfo;

};