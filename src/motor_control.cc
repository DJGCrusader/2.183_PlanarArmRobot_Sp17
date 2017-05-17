#include <stdio.h>
#include <iostream>
#include "../include/cmaxonmotor.h"
#include <unistd.h>
#include <chrono>
#include <termios.h>
#include <fstream>  //THIS IS TO WRITE FILE
#include <cmath>
#include <queue>

#define NB_ENABLE 0
#define NB_DISABLE 1
#define CTRLFREQ 38.0
#define CTRLPERIOD (1.0/CTRLFREQ)
#define LATEPERIOD (CTRLPERIOD/2.0)
#define KT0 217.0 //milliNewton-meter per Amp
#define KT1 70.5
#define KT2 70.5
#define KV0 4.608 //mA per milliNewton-Meter
#define KV1 14.180
#define KV2 14.180
#define PI 3.1415926
#define ENC2RAD (2*PI/72.0)

double KJOINT[3];
double BJOINT[3];
double KPOS[3];
double KVEL[3];
double STARTPOINT[2];
double ENDPOINT[2];
double L1;
double L2;
double delayVel;
double delayPos;

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

double* ReflexFeedback(double* thetaDesired, double* omegaDesired, double* thetaCurrent, double* omegaCurrent){    
    static double thetaEP[3];
    // ThetaCurrentPrev=thetaCurrent;
    // OmegaCurrentPrev=omegaCurrent;
    
    thetaEP[0]=thetaDesired[0] + (thetaDesired[0]-thetaCurrent[0])*KPOS[0] + (omegaDesired[0]-omegaCurrent[0])*KVEL[0];
    thetaEP[1]=thetaDesired[1] + (thetaDesired[1]-thetaCurrent[1])*KPOS[1] + (omegaDesired[1]-omegaCurrent[1])*KVEL[1];
    thetaEP[2]=thetaDesired[2] + (thetaDesired[2]-thetaCurrent[2])*KPOS[2] + (omegaDesired[2]-omegaCurrent[2])*KVEL[2];
    return thetaEP;
}

double* EPModel(double* thetaDesired, double* omegaDesired, double* thetaCurrent, double* omegaCurrent, double* thetaDelayed, double* omegaDelayed){
    static double torqueDesired [3];
    static double* thetaEP;
    thetaEP = ReflexFeedback(thetaDesired, omegaDesired, thetaDelayed, omegaDelayed);
    torqueDesired[0]=(( thetaEP[0]-thetaCurrent[0])*KJOINT[0]  - (omegaCurrent[0]*BJOINT[0])) + (( thetaEP[1]-thetaCurrent[1])*KJOINT[2]  - (omegaCurrent[1]*BJOINT[2]));
    torqueDesired[1]=(( thetaEP[1]-thetaCurrent[1])*KJOINT[1]  - (omegaCurrent[1]*BJOINT[1])) + (( thetaEP[0]-thetaCurrent[0])*KJOINT[2]  - (omegaCurrent[0]*BJOINT[2]));
    torqueDesired[2]=( thetaEP[2]-thetaCurrent[2])*KJOINT[2]  - (omegaCurrent[2]*BJOINT[2]);
    return torqueDesired;
}

double* anglesToHandPos(double th1, double th2){
    static double handPos[2];
    handPos[0] = cos(th1)*L1+cos(th1+th2)*L2;
    handPos[1] = sin(th1)*L1+sin(th1+th2)*L2;
    return handPos;
}

double* handPosToAngles(double x, double y){
    static double thetas[2];
    double alphas[3];
    alphas[0]= atan(y/x);
    alphas[1] = acos((pow(x, 2)+ pow(y, 2)+pow(L1, 2)-pow(L2, 2))/(2*L1*sqrt(pow(x, 2)+pow(y, 2))));
    alphas[2] = acos((pow(x, 2)+ pow(y, 2)+pow(L2, 2)-pow(L1, 2))/(2*L2*sqrt(pow(x, 2)+pow(y, 2))));
    thetas[0] = alphas[0]-alphas[1];
    thetas[1] = alphas[1]+alphas[2];
    return thetas;
}

double* minJerkTrajectory(double tt, double duration){
    static double minimumJerkTrajectory_theta_omega[4]; // th1 th2 position, th1 th2 velocity
    double minimumJerkTrajectory_x_v[4]; //x, y position, x, y velocity 
    
    if(tt> duration){
        double* endPos_theta_omega = minJerkTrajectory(duration, duration); //end of movement position and velocity   
        minimumJerkTrajectory_theta_omega[0]= endPos_theta_omega[0]; 
        minimumJerkTrajectory_theta_omega[1]= endPos_theta_omega[1]; 
        minimumJerkTrajectory_theta_omega[2]=  0;
        minimumJerkTrajectory_theta_omega[3]=  0;
    }
    else {
        minimumJerkTrajectory_x_v[0]= STARTPOINT[0] + (ENDPOINT[0]-STARTPOINT[0])*(10*pow((tt/duration), 3) - 15*pow((tt/duration), 4) + 6*pow((tt/duration),5)); 
        minimumJerkTrajectory_x_v[1]= STARTPOINT[1] + (ENDPOINT[1]-STARTPOINT[1])*(10*pow((tt/duration), 3) - 15*pow((tt/duration), 4) + 6*pow((tt/duration),5)); 

        minimumJerkTrajectory_x_v[2]=  (ENDPOINT[0]-STARTPOINT[0])*(1/duration)*(30*pow((tt/duration), 2) - 60*pow((tt/duration), 3) + 30*pow((tt/duration), 4)); 
        minimumJerkTrajectory_x_v[3]=  (ENDPOINT[1]-STARTPOINT[1])*(1/duration)*(30*pow((tt/duration), 2) - 60*pow((tt/duration), 3) + 30*pow((tt/duration), 4)); 
        double* endPos;
        endPos = handPosToAngles(minimumJerkTrajectory_x_v[0], minimumJerkTrajectory_x_v[1]);
        minimumJerkTrajectory_theta_omega[0]= endPos[0]; 
        minimumJerkTrajectory_theta_omega[1]= endPos[1]; 
        double* endVel;
        endVel = handPosToAngles(minimumJerkTrajectory_x_v[2], minimumJerkTrajectory_x_v[3]);
        minimumJerkTrajectory_theta_omega[2]=  endVel[0]; 
        minimumJerkTrajectory_theta_omega[3]=  endVel[0];
    }
    return minimumJerkTrajectory_theta_omega;
}

int main(int argc, char *argv[])
{
    char inChar;
    int inCheck=0;

    bool running = 1;

    // ------------------------------------------------------------------------------Definitions
    delayVel = 0.040;
    delayPos = 0.065; //artificially increased by an order of magnitude

    //Reflex Params
    KVEL[0] = .2*0.3; //Ranges 0.3 to 0.6, Vel delay 40ms for 0.3, 26ms for 0.6. 
    KVEL[1] = .2*0.3;
    KVEL[2] = .2*0.3;
    KPOS[0] = .2*1.4; //1.4; //Ranges from 1.4 to 2.5, Pos delay 65ms
    KPOS[1] = .2*1.4; //1.4;
    KPOS[2] = 1.4; //1.4;


    KJOINT[0] = 1000*1.6; //4; //N*m per rad, slow. 64 for fast movements
    KJOINT[1] = 1000*.8; //4;
    KJOINT[2] = 1000*.8; //4;
    BJOINT[0] = 1400*KJOINT[0];//1000*3.75; ////.89; //0.89; // N*m per rad per sec, 0.707 zeta
    BJOINT[1] = 1400*KJOINT[1];//1000*1.764; //0.89;
    BJOINT[2] = 1400*KJOINT[2];//1000*0.48; ////0.89;

    //target positions in meters
    //double target_x[4];
    //double target_y[4];
    double target_x[4] = {0, 0, -0.140, 0.325};
    double target_y[4] = {0.273, 0.5, 0.55, 0.273}; //0.584
    //arm lengths in meters
    L1 = 0.279;
    L2 = 0.257;
    int start = 0;         //index of start target, 0 - 3
    int end = 1;           //index of end target, 0 -3
    double movement_duration = 5;      //duration in seconds

    double* theta_start;
    theta_start = handPosToAngles(target_x[start], target_y[start]);
    double* theta_end;
    theta_end = handPosToAngles(target_x[end], target_y[end]);
    STARTPOINT[0]=target_x[start]; //x and y of start target
    STARTPOINT[1]=target_y[start];
    ENDPOINT[0]= target_x[end]; //x and y of end target
    ENDPOINT[1]= target_y[end];

    sleep(1);
    cout << "Press <Enter> to begin..." << endl;
    getchar();
    nonblock(NB_ENABLE);

    double* minJerk_theta_omega;

    double thetaCurrent[3];
    double thetaDelayed[3];
    queue <double> qThetaDelayed0;
    queue <double> qThetaDelayed1;
    int thetaDelayNum = int(delayPos/CTRLPERIOD+0.5);

    double omegaCurrent[3];
    double omegaDelayed[3];
    queue <int> qOmegaDelayed0;
    queue <int> qOmegaDelayed1;
    int omegaDelayNum = int(delayVel/CTRLPERIOD+0.5);

    short currentCurrent[3];
    double* torqueDesired;
    
    double thetaDesired[3];
    thetaDesired[0] = PI/4.0; //17*ENC2RAD; //minjerk(time);
    thetaDesired[1] = PI/4.0; //-21*ENC2RAD;
    thetaDesired[2] = 0;

    double omegaDesired[3];
    omegaDesired[0] = 0; //minjerk(time);
    omegaDesired[1] = 0;
    omegaDesired[2] = 0;
   
    short currentDesired[3];
    currentDesired[0]=0;
    currentDesired[1]=0;
    currentDesired[2]=0;

    // ------------------------------------------------------------------------------Setup
    sleep(1);
    cout << "Press <Enter> to begin..." << endl;
    getchar();
    nonblock(NB_ENABLE);

    //Set up data logging
    ofstream myfile;
    myfile.open ("results.txt");
    myfile << "time thetaCurrent1 thetaDesired1 omegaCurrent1 omegaDesired1 thetaCurrent2 thetaDesired2 omegaCurrent2 omegaDesired2\n";

    //  Setup Maxon Motor Objects
    CMaxonMotor motor;
    motor.CloseAllDevice();
    motor.ActiviateAllDevice();
    motor.SetCurrentModeAll();
    motor.SetCurrentAll(currentDesired);

    //Populate delayed queue
    motor.GetCurrentVelAllDevice(omegaCurrent);
    for(int i = 0; i < omegaDelayNum; i++){
        qOmegaDelayed0.push(omegaCurrent[0]);
        qOmegaDelayed1.push(omegaCurrent[1]);
    }
            
    motor.GetCurrentPositionAllDevice(thetaCurrent);
    for(int i = 0; i < thetaDelayNum; i++){
        qThetaDelayed0.push(thetaCurrent[0]);
        qThetaDelayed1.push(thetaCurrent[1]);
    }

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
                cout<<"SLOW! Loop took "<< over << " seconds over" << endl;
            }
            //  For timing
            startT = std::chrono::high_resolution_clock::now();
            
            // Update Desired

            minJerk_theta_omega = minJerkTrajectory(totalElapsed.count(), movement_duration);
            // thetaDesired[0] = minJerk_theta_omega[0]; //minjerk(time);
            // thetaDesired[1] = minJerk_theta_omega[1];
            // thetaDesired[2] = 0;
            // omegaDesired[0] = minJerk_theta_omega[2]; //minjerk(time);            
            // omegaDesired[1] = minJerk_theta_omega[3];
            // omegaDesired[2] = 0;

            // Sensing and time delay
            omegaDelayed[0] = qOmegaDelayed0.front();
            qOmegaDelayed0.pop();
            omegaDelayed[1] = qOmegaDelayed1.front();
            qOmegaDelayed1.pop();
            thetaDelayed[0] = qThetaDelayed0.front();
            qThetaDelayed0.pop();
            thetaDelayed[1] = qThetaDelayed1.front();
            qThetaDelayed1.pop();

            motor.GetCurrentVelAllDevice(omegaCurrent);
            qOmegaDelayed0.push(omegaCurrent[0]);
            qOmegaDelayed1.push(omegaCurrent[1]);
            motor.GetCurrentPositionAllDevice(thetaCurrent);
            qThetaDelayed0.push(thetaCurrent[0]);
            qThetaDelayed1.push(thetaCurrent[1]);
            motor.GetCurrentAll(currentCurrent);
            cout << (360*thetaCurrent[0]/(2*PI)) << " " << (360*thetaCurrent[1]/(2*PI)) << endl;
            cout << (360*omegaCurrent[0]/(2*PI)) << " " << (360*omegaCurrent[1]/(2*PI)) << endl;

            //  Control
            //torqueDesired = randomTest();
            torqueDesired = EPModel(thetaCurrent, omegaCurrent, thetaDesired, omegaDesired, omegaDelayed, thetaDelayed);

            currentDesired[0] = KV0*torqueDesired[0]/2;
            currentDesired[1] = KV1*torqueDesired[1]/2;
            currentDesired[2] = KV2*torqueDesired[2]/2;
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