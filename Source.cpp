#include <Windows.h>
#include <iostream>
#include <conio.h>
#include <vector>
#include <string>




using namespace std;

//protocol
//Grasp type : 0 = palmar, 1 = lateral
//object type : 0 = box, 1 = cylinder, 2 = sphere
//object size : 3 numbers each consiting of 3 numbers

//We can change buffer and message size when getting in more information 
void ComWrite(string message) {

    char chmessage[4] = {'0', '0', '0', '0'};

    for (size_t i = 0; i < message.length(); i++) {
        
        chmessage[i] = message[i];
        
    }
    
    cout << chmessage[0] << endl;
    cout << chmessage[1] << endl;
    cout << chmessage[2] << endl;
    cout << chmessage[3] << endl;

    //vector<char> buffer[] = { message };
    const char buffer[4] = { chmessage[0], chmessage[1], chmessage[2], chmessage[3] };
    

    DCB dcb = { 0 };
    DWORD bytesWritten;

    HANDLE sHandler;

    sHandler = CreateFile(L"COM5", GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);

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

    int position = 90;

    
    int state = 1;
    // 1 = open and 0 = closed

    while (1) {


        char ch;

        if (GetAsyncKeyState(0x53)) {
            ch = 's';
        }
        else if (GetAsyncKeyState(0x44)) {
            ch = 'd';
        }
        else ch = _getch();

        // Control Protocol : ID - Angle


        //Simulate camera rotation
        if (ch == 'q') {
            position = position + 1;
            cout << position << endl;
        }
        if (ch == 'e') {
            position = position - 1;
            cout << position << endl;
        }


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

                result = result + to_string(position);

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