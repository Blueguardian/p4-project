#include "UDPCom.h"


#include <iostream>
#include <ws2tcpip.h>
#include <string>

#pragma comment (lib, "ws2_32.lib")

using namespace std;

sockaddr_in server;
SOCKET out;

void UDPCom::UDPInit() {

    //startup winsock
    WSADATA data;
    WORD version = MAKEWORD(2, 2);
    int wsOk = WSAStartup(version, &data);
    if (wsOk != 0) {
        cout << "Cant start winsock!" << wsOk;
        return;
    }

    // Create a hint structure for the server
    
    server.sin_family = AF_INET;
    server.sin_port = htons(54000);
    inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);

    //socket creation
    out = socket(AF_INET, SOCK_DGRAM, 0);
}


void UDPCom::UDPSend(std::string output)
{
    //write out to socket
    int sendOk = sendto(out, output.c_str(), output.size() + 1, 0, (sockaddr*)&server, sizeof(server));

    if (sendOk == SOCKET_ERROR) {
        cout << "failure" << WSAGetLastError() << endl;
    }

    //close socket
    closesocket(out);

    //close down winsocket
    WSACleanup();

}