#include <stdio.h>
#include <iostream>
#include "../include/cmaxonmotor.h"
#include <unistd.h>
#include <chrono>
#include <termios.h>

#define NB_ENABLE 0
#define NB_DISABLE 1
#define CTRLFREQ 100
#define CTRLPERIOD 1/CTRLFREQ

using namespace std;


//Helper Function to read user input while still running loop, for quitting
void nonblock(int state)
{
    struct termios ttystate;
 
    //get the terminal state
    tcgetattr(STDIN_FILENO, &ttystate);
 
    if (state==NB_ENABLE)
    {
        //turn off canonical mode
        ttystate.c_lflag &= ~ICANON;
        //minimum of number input read.
        ttystate.c_cc[VMIN] = 1;
    }
    else if (state==NB_DISABLE)
    {
        //turn on canonical mode
        ttystate.c_lflag |= ICANON;
    }
    //set the terminal attributes.
    tcsetattr(STDIN_FILENO, TCSANOW, &ttystate);
 
}

//Check user input while still running loop, for quitting
int kbhit()
{
    struct timeval tv;
    fd_set fds;
    tv.tv_sec = 0;
    tv.tv_usec = 0;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds); //STDIN_FILENO is 0
    select(STDIN_FILENO+1, &fds, NULL, NULL, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

int main(int argc, char *argv[])
{
    char inChar;
    int inCheck=0;
    bool running = 1;

    sleep(1);
    cout << "Press <Enter> to begin..." << endl;
    getchar();
    nonblock(NB_ENABLE);

    //  Setup Maxon Motor Objects
    CMaxonMotor motor;
    motor.CloseAllDevice();
    motor.ActiviateAllDevice();
    motor.SetCurrentModeAll();
    short currentDesired[3];
    currentDesired[0]=0;
    currentDesired[1]=0;
    currentDesired[2]=0;
    short currentAll[3];
    motor.SetCurrentAll(currentDesired);

    //  Record start time
    std::chrono::time_point<std::chrono::high_resolution_clock>
                        start = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock>
                        finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;


    ///////////////////////////////////////////////////////////////////////////////////////// MAIN LOOP
    while(running){
        //  Timing to kep a certain frequency
        finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;

        if(elapsed.count() >= CTRLPERIOD){
            //  For timing
            start = std::chrono::high_resolution_clock::now();

            //  Control
            currentDesired[0]=rand()%500 - 250;
            currentDesired[1]=rand()%500 - 250;
            currentDesired[2]=rand()%500 - 250;
            motor.SetCurrentAll(currentDesired);

            //Quit when user presses 'q'
            inCheck=kbhit();
            if (inCheck!=0)
            {
                inChar=fgetc(stdin);
                if (inChar=='q'){
                    cout<<"\nQuitting!"<< endl;
                    running=0;
                    inCheck = 1;
                    nonblock(NB_DISABLE);
                }else{
                    running=1;
                    inCheck = 0;
                }
            }

        }
    }
    //////////////////////////////////////////////////////////////////////////////////////////// MAIN LOOP END


    currentDesired[0]=0;
    currentDesired[1]=0;
    currentDesired[2]=0;
    // motor.SetCurrentAll(currentDesired);
    // motor.DisableAllDevice();
    return 0;
}