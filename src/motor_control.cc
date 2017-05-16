#include <stdio.h>
#include <iostream>
#include "../include/cmaxonmotor.h"
#include <unistd.h>
#include <chrono>
#include <termios.h>
#include <fstream>  //THIS IS TO WRITE FILE

#define NB_ENABLE 0
#define NB_DISABLE 1
#define CTRLFREQ 10.0
#define CTRLPERIOD 1.0/CTRLFREQ
#define LATEPERIOD CTRLPERIOD/4.0
#define KT0 217.0 //milliNewton-meter per Amp
#define KT1 217.0
#define KT2 70.5
#define KV0 4.608 //mA per milliNewton-Meter
#define KV1 4.608
#define KV2 14.180

double KJOINT[3];
double BJOINT[3];
double KPOS[3];
double KVEL[3];


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

double* randomTest(){
    double torqueDesired[3];
    torqueDesired[0]=0; //rand()%400;//test: apply random currents
    torqueDesired[1]=0; //rand()%400; //UNITS: milliNewton-Meters
    torqueDesired[2]=rand()%400 - 200;

    return torqueDesired;
}

double* ReflexFeedback(int* thetaDesired, int* omegaDesired, int* thetaCurrent, int* omegaCurrent){    
    double thetaEP[3];
    // ThetaCurrentPrev=thetaCurrent;
    // OmegaCurrentPrev=omegaCurrent;
    
    thetaEP[0]=thetaDesired[0] + (thetaDesired[0]-thetaCurrent[0])*KPOS[0] + (omegaDesired[0]-omegaCurrent[0])*KVEL[0];
    thetaEP[1]=thetaDesired[1] + (thetaDesired[1]-thetaCurrent[1])*KPOS[1] + (omegaDesired[1]-omegaCurrent[1])*KVEL[1];
    thetaEP[2]=thetaDesired[2] + (thetaDesired[2]-thetaCurrent[2])*KPOS[2] + (omegaDesired[2]-omegaCurrent[2])*KVEL[2];
    return thetaEP;
}

double* EPModel(int* thetaDesired, int* omegaDesired, int* thetaCurrent, int* omegaCurrent){
    double torqueDesired [3];
    double* thetaEP;
    thetaEP = ReflexFeedback(thetaDesired, omegaDesired, thetaCurrent, omegaCurrent);
    torqueDesired[0]=-1*(( thetaEP[0]-thetaCurrent[0])*KJOINT[0]  - (omegaCurrent[0]*BJOINT[0]));
    torqueDesired[1]=-1*(( thetaEP[1]-thetaCurrent[1])*KJOINT[1]  - (omegaCurrent[1]*BJOINT[1]));
    torqueDesired[2]=( thetaEP[2]-thetaCurrent[2])*KJOINT[2]  - (omegaCurrent[2]*BJOINT[2]);
    return torqueDesired;
}


int main(int argc, char *argv[])
{
    //Set up data logging
    ofstream myfile;
    myfile.open ("results.txt");
    myfile << "time thetaCurrent1 thetaDesired1 omegaCurrent1 omegaDesired1 thetaCurrent2 thetaDesired2 omegaCurrent2 omegaDesired2\n\n";

    char inChar;
    int inCheck=0;
    bool running = 1;

    //Reflex Params
    KVEL[0] = 0.3; //Ranges 0.3 to 0.6, Vel delay 40ms for 0.3, 26ms for 0.6. 
    KVEL[1] = 0.3;
    KVEL[2] = 0.3;
    KPOS[0] = 2.4; //1.4; //Ranges from 1.4 to 2.5, Pos delay 65ms
    KPOS[1] = 1.4; //1.4;
    KPOS[2] = 1.4; //1.4;

    BJOINT[0] = 0.89; // N*m per rad per sec, 0.707 zeta
    BJOINT[1] = 0.89;
    BJOINT[2] = 0.89;
    KJOINT[0] = 16; //4; //N*m per rad, slow. 64 for fast movements
    KJOINT[1] = 8; //4;
    KJOINT[2] = 8; //4;

    sleep(1);
    cout << "Press <Enter> to begin..." << endl;
    getchar();
    nonblock(NB_ENABLE);

    //  Setup Maxon Motor Objects
    CMaxonMotor motor;
    motor.CloseAllDevice();
    motor.ActiviateAllDevice();
    motor.SetCurrentModeAll();

    int thetaCurrent[3];
    int omegaCurrent[3];
    int thetaDesired[3];
    thetaDesired[0] = 17; //minjerk(time);
    thetaDesired[1] = -21;
    thetaDesired[2] = 0;

    int omegaDesired[3];
    omegaDesired[0] = 0; //minjerk(time);            
    omegaDesired[1] = 0;
    omegaDesired[2] = 0;
    
    short currentCurrent[3];

    double* torqueDesired;
   
    short currentDesired[3];
    currentDesired[0]=0;
    currentDesired[1]=0;
    currentDesired[2]=0;
    motor.SetCurrentAll(currentDesired);

    //  Record start time
    std::chrono::time_point<std::chrono::high_resolution_clock>
                        totalStart = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock>
                        start = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock>
                        finish = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finish - start;
    std::chrono::duration<double> totalElapsed = finish - totalStart;
    double over = 0;

    cout << "Running. Press <q> to Quit!" << endl;

    ///////////////////////////////////////////////////////////////////////////////////////// MAIN LOOP
    while(running){
        //  Timing to kep a certain frequency
        finish = std::chrono::high_resolution_clock::now();
        elapsed = finish - start;
        totalElapsed = finish - totalStart;

        if(elapsed.count() >= CTRLPERIOD){

            over = elapsed.count()-(CTRLPERIOD);
            if( over > LATEPERIOD){
                cout<<"SLOW! Loop took "<< over << " seconds" << endl;
            }
            //  For timing
            start = std::chrono::high_resolution_clock::now();
            
            // Update Desired
            // thetaDesired[0] = 0; //minjerk(time);
            // thetaDesired[1] = 0;
            // thetaDesired[2] = 0; 
            // omegaDesired[0] = 0; //minjerk(time);            
            // omegaDesired[1] = 0;
            // omegaDesired[2] = 0;


            // Sensing
            motor.GetCurrentVelAllDevice(omegaCurrent);
            motor.GetCurrentPositionAllDevice(thetaCurrent);
            motor.GetCurrentAll(currentCurrent);
            cout << thetaCurrent[0] << " " << thetaCurrent[1] << endl;

            //  Control
            //torqueDesired = randomTest();
            torqueDesired = EPModel(thetaCurrent, omegaCurrent, thetaDesired, omegaDesired);

            currentDesired[0] = KV0*torqueDesired[0];
            currentDesired[1] = KV1*torqueDesired[1];
            currentDesired[2] = KV2*torqueDesired[2];
            motor.SetCurrentAll(currentDesired);

            //  Logging 
            myfile << totalElapsed.count() << " " << thetaCurrent[0] << " " << thetaDesired[0] << " "
                                                << omegaCurrent[0] << " " << omegaDesired[0] << " "
                                                << thetaCurrent[1] << " " << thetaDesired[1] << " "
                                                << omegaCurrent[1] << " " << omegaDesired[1] << "\n";

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


    myfile.close();
    currentDesired[0]=0;
    currentDesired[1]=0;
    currentDesired[2]=0;
    motor.SetCurrentAll(currentDesired);
    motor.DisableAllDevice();
    return 0;
}