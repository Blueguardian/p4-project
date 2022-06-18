#include <iostream>
#include <ws2tcpip.h>
#include <string.h>
#include <conio.h>

#pragma comment (lib, "ws2_32.lib")

using namespace std;

char ch;

uint8_t micpackage[9];

uint8_t byte0;
uint8_t byte1;
uint8_t byte2;
uint8_t byte3;
uint8_t byte4;
uint8_t byte5;
uint8_t byte6;
uint8_t byte7;
uint8_t byte8;

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
    server.sin_port = htons(8051);
    inet_pton(AF_INET, "127.0.0.1", &server.sin_addr);

    //socket creation
    SOCKET out = socket(AF_INET, SOCK_DGRAM, 0);

    while (1) {
        //write out to socket
        //string s(argv[1]);

        ch = _getch();

        if (ch == 's') {
            //"S" has ben pressed
            micpackage[0] = 1;
            micpackage[1] = 0;
            micpackage[2] = 0;
            micpackage[3] = 0;
            micpackage[4] = 0;
            micpackage[5] = 200;
            micpackage[6] = 0;
            micpackage[7] = 0;
            micpackage[8] = 0;

            cout << "S" << endl;
        }

        else if (ch == 'd') {
            //"D" has been pressed
            micpackage[0] = 1;
            micpackage[1] = 0;
            micpackage[2] = 0;
            micpackage[3] = 0;
            micpackage[4] = 0;
            micpackage[5] = 0;
            micpackage[6] = 200;
            micpackage[7] = 0;
            micpackage[8] = 0;

            cout << "D" << endl;
        }


        else if (ch == 'a') {
            //"D" has been pressed
            micpackage[0] = 1;
            micpackage[1] = 0;
            micpackage[2] = 0;
            micpackage[3] = 0;
            micpackage[4] = 0;
            micpackage[5] = 0;
            micpackage[6] = 0;
            micpackage[7] = 0;
            micpackage[8] = 0;

            cout << "A" << endl;
        }

        else if (ch == 'z') {
            //"D" has been pressed
            micpackage[0] = 1;
            micpackage[1] = 100;
            micpackage[2] = 0;
            micpackage[3] = 0;
            micpackage[4] = 0;
            micpackage[5] = 0;
            micpackage[6] = 0;
            micpackage[7] = 0;
            micpackage[8] = 0;

            cout << "z" << endl;
        }

        else if (ch == 'x') {
            //"D" has been pressed
            micpackage[0] = 1;
            micpackage[1] = 0;
            micpackage[2] = 100;
            micpackage[3] = 0;
            micpackage[4] = 0;
            micpackage[5] = 0;
            micpackage[6] = 0;
            micpackage[7] = 0;
            micpackage[8] = 0;

            cout << "x" << endl;
        }

        int sendOk = sendto(out, (char*)micpackage, 9, 0, (sockaddr*)&server, sizeof(server));

        if (sendOk == SOCKET_ERROR) {
            cout << "failure" << WSAGetLastError() << endl;
        }

    }
    //close socket
    closesocket(out);

    //close down winsocket
    WSACleanup();

}
