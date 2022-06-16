#include <iostream>
#include <ws2tcpip.h>
#include <string.h>

#pragma comment (lib, "ws2_32.lib")

using namespace std;

string micpackage;

string byte0 ;
string byte1 ;
string byte2 ;
string byte3 ;
string byte4 ;
string byte5 ;
string byte6 ;
string byte7 ;

void CreatePack() {
    micpackage = byte0 + byte1 + byte2 + byte3 + byte4 + byte5 + byte6 + byte7;
}


void main(int argc, char* argv[]) //ability to pass in command line option
{

    //Define package
    


    //startup winsock
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

    while (1) {
        //write out to socket
        //string s(argv[1]);
        cin >> byte0;
        cin >> byte1;
        cin >> byte2;
        cin >> byte3;
        cin >> byte4;
        cin >> byte5;
        cin >> byte6;
        cin >> byte7;

        CreatePack();

        int sendOk = sendto(out, micpackage.c_str(), micpackage.size() + 1, 0, (sockaddr*)&server, sizeof(server));

        if (sendOk == SOCKET_ERROR) {
            cout << "failure" << WSAGetLastError() << endl;
        }

    }
    //close socket
    closesocket(out);

    //close down winsocket
    WSACleanup();

}