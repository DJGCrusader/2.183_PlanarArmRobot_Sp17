#include <stdio.h>
#include <iostream>
#include "../include/cmaxonmotor.h"
#include <unistd.h>
#include <chrono>
#include <termios.h>
#include <fstream>  //THIS IS TO WRITE FILE
#include <cmath>

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
double STARTPOINT[2];
double ENDPOINT[2];


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
    static double torqueDesired[3];
    torqueDesired[0]=0; //rand()%400;//test: apply random currents
    torqueDesired[1]=0; //rand()%400; //UNITS: milliNewton-Meters
    torqueDesired[2]=rand()%400 - 200;

    return torqueDesired;
}

double* ReflexFeedback(int* thetaDesired, int* omegaDesired, int* thetaCurrent, int* omegaCurrent){    
    static double thetaEP[3];
    // ThetaCurrentPrev=thetaCurrent;
    // OmegaCurrentPrev=omegaCurrent;
    
    thetaEP[0]=thetaDesired[0] + (thetaDesired[0]-thetaCurrent[0])*KPOS[0] + (omegaDesired[0]-omegaCurrent[0])*KVEL[0];
    thetaEP[1]=thetaDesired[1] + (thetaDesired[1]-thetaCurrent[1])*KPOS[1] + (omegaDesired[1]-omegaCurrent[1])*KVEL[1];
    thetaEP[2]=thetaDesired[2] + (thetaDesired[2]-thetaCurrent[2])*KPOS[2] + (omegaDesired[2]-omegaCurrent[2])*KVEL[2];
    return thetaEP;
}

double* EPModel(int* thetaDesired, int* omegaDesired, int* thetaCurrent, int* omegaCurrent){
    static double torqueDesired [3];
    double* thetaEP;
    thetaEP = ReflexFeedback(thetaDesired, omegaDesired, thetaCurrent, omegaCurrent);
    torqueDesired[0]=-1*(( thetaEP[0]-thetaCurrent[0])*KJOINT[0]  - (omegaCurrent[0]*BJOINT[0]));
    torqueDesired[1]=-1*(( thetaEP[1]-thetaCurrent[1])*KJOINT[1]  - (omegaCurrent[1]*BJOINT[1]));
    torqueDesired[2]=( thetaEP[2]-thetaCurrent[2])*KJOINT[2]  - (omegaCurrent[2]*BJOINT[2]);
    return torqueDesired;
}


double* minJerkTrajectory(double time, double duration){
    
    static double minimumjerktrajectory_theta_omega[4];
    
    if(time> duration){
        double* endPos = minJerkTrajectory(duration, duration); //end of movement position and velocity
        minimumjerktrajectory_theta_omega[0]= endPos[0]; 
        minimumjerktrajectory_theta_omega[1]= endPos[1]; 

        minimumjerktrajectory_theta_omega[2]=  0;
        minimumjerktrajectory_theta_omega[3]=  0;
    }
    else {
        minimumjerktrajectory_theta_omega[0]= STARTPOINT[0] + (ENDPOINT[0]-STARTPOINT[0])*(10*pow((time/duration), 3) - 15*pow((time/duration), 4) + 6*pow((time/duration),5)); 
        minimumjerktrajectory_theta_omega[1]= STARTPOINT[1] + (ENDPOINT[1]-STARTPOINT[1])*(10*pow((time/duration), 3) - 15*pow((time/duration), 4) + 6*pow((time/duration),5)); 

        minimumjerktrajectory_theta_omega[2]=  (ENDPOINT[0]-STARTPOINT[0])*(1/duration)*(30*pow((time/duration), 2) - 60*pow((time/duration), 3) + 30*pow((time/duration), 4)); 
        minimumjerktrajectory_theta_omega[3]=  (ENDPOINT[1]-STARTPOINT[1])*(1/duration)*(30*pow((time/duration), 2) - 60*pow((time/duration), 3) + 30*pow((time/duration), 4)); 

    }
    return minimumjerktrajectory_theta_omega;
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

    //target positions in meters
    //double target_x[4];
    //double target_y[4];
    double target_x[4] = {0, 0, -0.140, 0.325};
    double target_y[4] = {0.273, 0.584, 0.522, 0.273};
    //arm lengths in meters
    double l1 = 0.279;
    double l2 = 0.257;
    int start = 0;         //index of start target, 0 - 3
    int end = 1;           //index of end target, 0 -3
    double movement_duration = 5;      //duration in seconds

    double alpha1_start = atan(target_y[start]/target_x[start]);
    double alpha2_start = acos((pow(target_x[start], 2)+ pow(target_y[start], 2)+pow(l1, 2)-pow(l2, 2))/(2*l1*sqrt(pow(target_x[start], 2)+pow(target_y[start], 2))));
    double alpha3_start = acos((pow(target_x[start], 2)+ pow(target_y[start], 2)+pow(l2, 2)-pow(l1, 2))/(2*l2*sqrt(pow(target_x[start], 2)+pow(target_y[start], 2))));
    double alpha1_end = atan(target_y[end]/target_x[end]);
    double alpha2_end = acos((pow(target_x[end], 2)+ pow(target_y[end], 2)+pow(l1, 2)-pow(l2, 2))/(2*l1*sqrt(pow(target_x[end], 2)+pow(target_y[end], 2))));
    double alpha3_end = acos((pow(target_x[end], 2)+ pow(target_y[end], 2)+pow(l2, 2)-pow(l1, 2))/(2*l2*sqrt(pow(target_x[end], 2)+pow(target_y[end], 2))));

    STARTPOINT[0]=alpha1_start - alpha2_start; //theta1 and theta2 of start target
    STARTPOINT[1]=alpha1_start+alpha3_start;
    ENDPOINT[0]=alpha1_end - alpha2_end; //theta1 and theta2 of end target
    ENDPOINT[1]= alpha1_end + alpha3_end;
    sleep(1);
    cout << "Press <Enter> to begin..." << endl;
    getchar();
    nonblock(NB_ENABLE);

    //  Setup Maxon Motor Objects
    CMaxonMotor motor;
    motor.CloseAllDevice();
    motor.ActiviateAllDevice();
    motor.SetCurrentModeAll();
    double* minJerk_theta_omega;

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
                        startT = std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock>
                        finishT = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = finishT - startT;
    std::chrono::duration<double> totalElapsed = finishT - totalStart;
    double over = 0;


    cout << "Running. Press <q> to Quit!" << endl;

    ///////////////////////////////////////////////////////////////////////////////////////// MAIN LOOP
    while(running){
        //  Timing to kep a certain frequency
        finishT = std::chrono::high_resolution_clock::now();
        elapsed = finishT - startT;
        totalElapsed = finishT - totalStart;

        if(elapsed.count() >= CTRLPERIOD){

            over = elapsed.count()-(CTRLPERIOD);
            if( over > LATEPERIOD){
                cout<<"SLOW! Loop took "<< over << " seconds" << endl;
            }
            //  For timing
            startT = std::chrono::high_resolution_clock::now();
            
            // Update Desired
            minJerk_theta_omega = minJerkTrajectory(elapsed.count(), movement_duration);
            thetaDesired[0] = minJerk_theta_omega[0];; //minjerk(time);
            thetaDesired[1] = minJerk_theta_omega[1];
            thetaDesired[2] = 0;
            omegaDesired[0] = minJerk_theta_omega[2]; //minjerk(time);            
            omegaDesired[1] = minJerk_theta_omega[3];
            omegaDesired[2] = 0;


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