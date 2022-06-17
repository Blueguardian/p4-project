//
// Created by mogens on 25/04/2022.
//

#define _CRT_SECURE_NO_WARNINGS



//#include <EMGHandler.cpp>
#include "Camerahandler.h"

#include <conio.h>



#include <Winsock2.h>
//#include <iostream>
#include <ws2tcpip.h>
//#include <string.h>

#pragma comment (lib, "ws2_32.lib")

using namespace royale;
using namespace sample_utils;
using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

void main(int argc, char* argv[])
{


    WSADATA data;
    WORD version = MAKEWORD(2, 2);
    int wsOk = WSAStartup(version, &data);
    if (wsOk != 0) {
        cout << "Cant start winsock!" << wsOk;
        return;
    }

    // Create a hint structure for the server
    sockaddr_in server;
    server.sin_family = AF_INET;
    server.sin_port = htons(54000);
    inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);

    //socket creation
    SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);








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
