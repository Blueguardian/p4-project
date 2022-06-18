#define _CRT_SECURE_NO_WARNINGS

#include "Camerahandler.h"
#include <conio.h>

using namespace royale;
using namespace sample_utils;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc, char* argv[])
{

    UDPCom udp;
    udp.UDPInit();
    

    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    Camerahandler camhandle;

    // this represents the main camera device object
    std::unique_ptr<ICameraDevice> cameraDevice;

    CameraManager manager;

    royale::Vector<royale::String> camlist(manager.getConnectedCameraList());
    cout << "Detected " << camlist.size() << " camera(s)." << endl;

    if (!camlist.empty())
    {
        cameraDevice = manager.createCamera(camlist[0]);
    }
    // the camera device is now available and CameraManager can be deallocated here

    // IMPORTANT: call the initialize method before working with the camera device
    auto status = cameraDevice->initialize();

    cameraDevice->setUseCase("MODE_5_45FPS_500"); //Register the correct usecase, Defualt useable, this one is better.
    cameraDevice->registerDataListener(&camhandle); // register a data listener
    cameraDevice->startCapture(); 

    

    while (!_kbhit()) // wait until a key is pressed
    {
       

    }
    cameraDevice->stopCapture();
    return 0;
}
