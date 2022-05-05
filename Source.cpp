#include <Windows.h>
#include <iostream>
#include <conio.h>
#include <vector>




using namespace std;

//protocol
//Grasp type : 0 = palmar, 1 = lateral
//object type : 0 = box, 1 = cylinder, 2 = sphere
//object size : 3 numbers each consiting of 3 numbers

void ComWrite(string message) {

    char chmessage[2];

    for (int i = 0; i < sizeof(chmessage); i++) {
        chmessage[i] = message[i];
    }
    


    //vector<char> buffer[] = { message };
    const char buffer[2] = { *chmessage };
    

    DCB dcb = { 0 };
    DWORD bytesWritten;

    HANDLE sHandler;

    sHandler = CreateFile(L"COM3", GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

    dcb.DCBlength = sizeof(dcb);

    GetCommState(sHandler, &dcb);
    dcb.BaudRate = 9600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(sHandler, &dcb);

    COMMTIMEOUTS timeout = { 0 };
    timeout.ReadIntervalTimeout = 50;
    timeout.ReadTotalTimeoutConstant = 50;
    timeout.ReadTotalTimeoutMultiplier = 50;
    timeout.WriteTotalTimeoutConstant = 50;
    timeout.WriteTotalTimeoutMultiplier = 10;

    SetCommTimeouts(sHandler, &timeout);

    WriteFile(sHandler, buffer, 1, &bytesWritten, NULL);

    CloseHandle(sHandler);

}



int main(int argc, char* argv[]) {

    //vector<char> result;
    string result;
    
    int state = 1;
    // 1 = open and 0 = closed

    while (1) {

        char ch = _getch();

        // Control Protocol : ID - Angle

        // Start analysis of surroundings

        if (ch == 'a') {

            if (state == 0) {
                cout << "Cannot analyse at this moment" << endl;
                result = "1";
                ComWrite(result);

            }
            else {
                cout << "Surroundings have been analysed!" << endl;

                result = "2";

                ComWrite(result);
            }

        }


        //open and close prosthesis

        if (ch == 's') {

            //if (state == 0) {
                //cout << "The gripper is already closed" << endl;
                //result = '1';
                //ComWrite(result);
            //}
            //else {
                cout << "The gripper has closed" << endl;
                state = 0;

                result = "3";

                ComWrite(result);
            //}

        }

        if (ch == 'd') {

            //if (state == 1) {
                //cout << "The gripper is already open" << endl;
                //result = '1';
                //ComWrite(result);
            //}
            //else {
                cout << "The gripper has opened" << endl;
                state = 1;

                result = "4";

                ComWrite(result);
            //}
        }



    }

}