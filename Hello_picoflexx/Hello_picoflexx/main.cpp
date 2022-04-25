//
// Created by mogens on 25/04/2022.
//

#include <include/Camerahandler.h>
#include <iosteam>

int main (int argc, char *argv[])
{
    // Windows requires that the application allocate these, not the DLL.
    PlatformResources resources;

    // This is the data listener which will receive callbacks.  It's declared
    // before the cameraDevice so that, if this function exits with a 'return'
    // statement while the camera is still capturing, it will still be in scope
    // until the cameraDevice's destructor implicitly de-registers the listener.
    CameraHandler camhandle;

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

    // retrieve the lens parameters from Royale
    LensParameters lensParameters;
    status = cameraDevice->getLensParameters(lensParameters);
    camhandle.setLensParameters(lensParameters);

    cameraDevice->registerDataListener(&camhandle);
    cameraDevice->startCapture();
    // register a data listener

    int currentKey = 0;
    while (currentKey != 27)
    {
        // wait until a key is pressed
        currentKey = waitKey(1);


        if (currentKey == 'd')
        {

            // toggle the undistortion of the image
            camhandle.toggleUndistort();
        }
    }
    cameraDevice->stopCapture();
    return 0;
}