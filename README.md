# PID Controller
Project 4 of term 2 of Udacity self-driving car nanodegree

## Setup

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Follow instructions [here](https://github.com/udacity/CarND-PID-Control-Project) to set up Ubuntu and Mac environments. Also, if you are using
one of these environments then copy src/main\_mac\_linux.cpp to src/main.cpp . 

Instructions for 
setting up a native Windows environment and Visual Studio project:

* follow steps 1-4 of the instructions [here](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/tree/master/VisualStudio)
* change install-windows.bat :
    - replace "EKF" with "Kidnapped-Vehicle"
    - delete row  "copy main.cpp ..\..\src" 
* run install-windows.bat 
* open solution in VisualStudio, remove from the solution all files from project 1, add all files from project 3
* run steps 6 and 7 from [here](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/tree/master/VisualStudio)

## Files
* src - folder of source files.
* videos - folder with the videos of several runs of the car with different values of parameters.
* ide_profiles - folder with IDE profiles for Ecliple and XCode.

## Usage
1. Set up project environment for your operating system.
2. Compile the files to create the main executable of the project (the name of executable depends on the operating system).
3. Run Term 2 Simulator
4. In the simulator, press "Next" button twice to switch to Project 4, then press "Select" button.
5. Run the main executable of the project.

The main executable can be run in three modes:
1. Driving with default parameters of PID controllers: just run the executable without any additional parameters
2. Driving with the specified values of PID controllers:

       <executable name> run <max throttle> <Kp> <Ki> <Kd> <s_Kp> <s_Ki> <s_Kd>

   where

       max throttle - maximal value of throttle during the run
       Kp - P parameter of PID controller of steering angle
       Ki - I parameter of PID controller of steering angle
       Kd - D parameter of PID controller of steering angle
       s_Kp - P parameter of PID controller of throttle
       s_Ki - I parameter of PID controller of throttle
       s_Kd - D parameter of PID controller of throttle

3. Tuning the parameters of PID controllers:

       <executable name> tune <max throttle> <Kp> <Ki> <Kd> <s_Kp> <s_Ki> <s_Kd> <tune_Kp> <tune_Ki> <tune_Kd> <tune_s_Kp> <tune_s_Ki> <tune_s_Kd> [cte | speed]    

    where

       max throttle - maximal value of throttle during the run
       Kp - initial value of P parameter of PID controller of steering angle
       Ki - initial value of I parameter of PID controller of steering angle
       Kd - initial value of D parameter of PID controller of steering angle
       s_Kp - initial value of P parameter of PID controller of throttle
       s_Ki - initial value of I parameter of PID controller of throttle
       s_Kd - initial value of D parameter of PID controller of throttle 
       tune_Kp - inidicator (0/1) if the value of Kp should be tuned 
       tune_Ki - indicator (0/1) if the value of Ki should be tuned 
       tune_Kd - indicator (0/1) if the value of Kd should be tuned 
       tune_s_Kp - inidicator (0/1) if the value of s_Kp should be tuned 
       tune_s_Ki - indicator (0/1) if the value of s_Ki should be tuned 
       tune_s_Kd - indicator (0/1) if the value of s_Kd should be tuned
       cte|speed - metric to optimize, "cte" - optimization of cte, "speed" - optimization of speed and cte.








