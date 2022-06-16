#include <Windows.h>
#include <iostream>
#include <conio.h>
#include <vector>
#include <string>




using namespace std;



int main(int argc, char* argv[]) {

    
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


        //Analyse surroundings
        if (ch == 'a') {


        }


        //open and close prosthesis

        if (ch == 's') {
            //Gripper closing

        }

        if (ch == 'd') {
            //Gripper opening
        }
    }
}