# PID Controller
Project 4 of term 2 of Udacity self-driving car nanodegree


[//]: # (Image References)

[image0]: ./images/pid.png "PID"
[image1]: ./images/cte.png "cte"
[image2]: ./images/pid_throttle.png "pid_throttle"
[image3]: ./images/objective.png "objective"

## Setup

This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Follow instructions [here](https://github.com/udacity/CarND-PID-Control-Project) to set up Ubuntu and Mac environments. Also, if you are using
one of these environments then copy src/main\_mac\_linux.cpp to src/main.cpp . 

Instructions for 
setting up a native Windows environment and Visual Studio project:

* follow steps 1-4 of the instructions [here](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/tree/master/VisualStudio)
* change install-windows.bat :
    - replace "EKF" with "PID-Controller"
    - delete row  "copy main.cpp ..\..\src" 
* run install-windows.bat 
* open solution in VisualStudio, remove from the solution all files from project 1, add all files from project 3
* run steps 6 and 7 from [here](https://github.com/fkeidel/CarND-Term2-ide-profile-VisualStudio/tree/master/VisualStudio)

## Files
* src - folder of source files.
* videos - folder with the videos of several runs of the car with different values of parameters.
* images - folder of auxiliary images that are used by README.
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
       s_Kp - P parameter of PID controller of speed
       s_Ki - I parameter of PID controller of speed
       s_Kd - D parameter of PID controller of speed

3. Tuning the parameters of PID controllers:

       <executable name> tune <max throttle> <Kp> <Ki> <Kd> <s_Kp> <s_Ki> <s_Kd> <tune_Kp> <tune_Ki> <tune_Kd> <tune_s_Kp> <tune_s_Ki> <tune_s_Kd> [cte | speed]    

    where

       max throttle - maximal value of throttle during the run
       Kp - initial value of P parameter of PID controller of steering angle
       Ki - initial value of I parameter of PID controller of steering angle
       Kd - initial value of D parameter of PID controller of steering angle
       s_Kp - initial value of P parameter of PID controller of speed
       s_Ki - initial value of I parameter of PID controller of speed
       s_Kd - initial value of D parameter of PID controller of speed 
       tune_Kp - inidicator (0/1) if the value of Kp should be tuned 
       tune_Ki - indicator (0/1) if the value of Ki should be tuned 
       tune_Kd - indicator (0/1) if the value of Kd should be tuned 
       tune_s_Kp - inidicator (0/1) if the value of s_Kp should be tuned 
       tune_s_Ki - indicator (0/1) if the value of s_Ki should be tuned 
       tune_s_Kd - indicator (0/1) if the value of s_Kd should be tuned
       cte|speed - metric to optimize, "cte" - optimization of cte, "speed" - joint optimization of speed and cte

## Reflection

The car is driven using two PID controllers, one for steering angle and another one for speed. The parameters of these controllers are tuned using a combination of manual tuning and twiddle algorithm. [This video](https://github.com/pechyony/pid-controller/blob/master/videos/all_parameters.mp4) shows the car driving using the tuned values of parameters. The car's speed varies between 20 and 55 mph, depending on car position and yaw angle. In the next three sections we describe our implementation of controllers and tuning of parameters in more details. 

### PID controllers of steering angle and speed
We used a standard PID controller for setting up the steering angle at time t:

![alt text][image0]

where cte(t) is a cross-track error at time t, defined as the difference between the desired yaw angle at time t and the actual yaw angle at the same time:

![alt text][image1]

The cross-track error is provided to us by simulator, which knows the desired yaw angle of car at each time. 

The speed of the car is proportional to throttle value. We implemented a variation of PID controller that controls the value of the throttle at time t:

![alt text][image2]

We set the maximal throttle value to 0.8. When the car drives in the optimal direction, cross-track errors are zero and the car drives with the maximal throttle value. PID controller reduces the throttle value when the car's direction deviates significantly from the desired direction. Also, in some cases, as discussed below, this PID controller can increase throttle and speed even if the cross-track error is not zero. 

Each of PID controllers has three parameters, Kp, Ki, Kd that can have non-negative values. In the next section we describe our procedure for tuning there parameters. Also in the last section we analyze influence of these parameters on car's driving.

### Tuning of parameters

Initially we tried manually multiple combinations of values of parameters to get sense what should be the order of magnitude of each parameter so that the car can complete several laps successfully. We found that we can achieve that with steering angle controller parameters Kp=1, Ki=0.01, Kd=40 and speed controller parameters Kp=1, Ki=0.001, Kd=0.01. Then we initialized Twiddle algorithm with these value and tuned them further.

We would like Twiddle algorithm to minimize the following metric:

![alt text][image3]

This metric penalizes driving with large cross-track error as well as slow driving. The above values of parameters had metric value of -4061.95.
After running Twiddle we found that the metric value can be reduced to -4805.99 when steering angle controller parameters are Kp=1, Ki=0.005, Kd=40 and speed controller parameters are Kp=1, Ki=0.001, Kd=0.00734279. With these values of parameters the root mean squared cross-track error was 0.394 and the average speed was 35.7 mph. These values of parameters are now used as a default values for driving a car. The log file of the tuning process can be found [here](https://github.com/pechyony/pid-controller/blob/master/tuning_log.txt). We also tried to do tuning of parameters with larger values of maximal throttle, but found that they don't improve the above metric. In the next paragraph we describe techical details of our implementation of Twiddle.

For each combination of candidate values of parameters our implementation of Twiddle algorithm drove car for 10000 iterations. If we detected that the car is stuck or has extremely large cross-track error then testing of the current value of parameters was stopped early, simulator was restarted and we proceeded to the next values of parameters. We initialized step value of each parameter to a half of its initial value. Also, we stopped tuning parameters when the sum of step values of all parameters was less than a half of the initial value of that sum.
 
Our implementation of Twiddle is generic and can be used with other controllers and objective functions.  

During the tuning of parameters we noticed that the driving performance depends on multiple software settings. For example, by running simulator with "Graphics quality"=Simple and relatively low screen resolution (e.g. 1024x768) we were able to drive car successfully at higher speeds.

### Influence of parameters 

#### PID controller of the steering angle

**Kp parameter** This parameter controls how fast the car's yaw angle changes towards the desired one. The effect of this parameter can be observed when the car drives in the wrong direction. A small value of Kp will result in a very slow change of direction towards the right one. In [this video](https://github.com/pechyony/pid-controller/blob/master/videos/no_P.mp4) we replaced the tuned value of Kp with zero, while keeping all other tuned parameter values. As expected, the car changed direction very slowly and as a result the car was stuck in the beginning of the run.
Also, a large value of Kp will lead to a very fast and large change of direction, which might result in a very wobbly driving. 

**Ki parameter** This parameter controls the correction of yaw angle as a result of car drift. If there is no drift then the sum of all cross-track errors is very close to zero and Ki parameter does not influence the steering angle. The drift occurs when the sum of cross-track error is significantly far from zero. In this case an optimal value of Ki will result in a change of car's direction that will reduce the drift effect. A very small value of Ki will result in a very slow change of direction when drift happens and a very large value of Ki will result in a very fast change of direction and wobbly driving. In [this video](https://github.com/pechyony/pid-controller/blob/master/videos/no_I.mp4) we replaced the tuned value of Ki with zero, while keeping all other tuned parameter values. Surprisingly, the car drives very slowly, with the average speed of 13.1 mph, but is able to complete the track. The successul completion of the track is probably because of the good values of other parameters, that are able to cancel the drift indirectly.  

**Kd parameter** This parameter controls the correction of yaw angle as a result of the change of cross track error between the current and the previous time. When Kd is very small or zero, PID controller overshoots the right direction and the car drives very wobbly. In [this video](https://github.com/pechyony/pid-controller/blob/master/videos/no_D.mp4) we show this effect by replacing the tuned value of Kd with 0 while keeping all other tuned parameter values. When Kd is very large the car drives very smoothly but slowly. 

#### PID controller of the speed

**Kp parameter** This parameter controls how fast the throttle is reduced when the car drives in the wrong direction. When Kp is small and the car starts to drive in the wrong direction the throttle will remain almost the same as previously. If the car makes sharp turn this might result in a car going off-track. In [this video](https://github.com/pechyony/pid-controller/blob/master/videos/no_s_P.mp4) we show this effect by replacing the tuned value of Kp with zero while keeping all other tuned parameter values. The car manages to complete the first lap, but goes off-track in the second lap. When Kp is very large then the throttle will be reduced significantly over the short period of time, which might lead to a very slow driving. 

**Ki parameter** This parameter controls how much the throttle is reduced when the car drifts. If Ki is very large then the car will drive very slowly when the drift is detected. When Ki is small the throttle will not be affected by the drift. This might result in wider turns and larger cross-track error. In [this video](https://github.com/pechyony/pid-controller/blob/master/videos/no_s_I.mp4) we show the car driving with Ki set to zero while keeping all other tuned parameter values. The car was able to complete successfully multiple laps, but had larger root mean squared cross-track error, 0.49, than the one with the tuned value of Ki. Also the car had larger average speed, 37.82 mph, than the one with the tuned value of Ki. Overall the metric value with Ki=0 was -4211.5, which is higher than the one with tuned Ki. 

**Kd parameter** This parameter controls the "aggressiveness" of the driving. When this parameter is large, the car slows down when the current cross-track error is larger than the previous one and speeds up when the current cross-track error is smaller than the previous one. In the former case controller assumes that the car drives in the wrong direction and applies additional reduction to the speed. Similarly, in the latter case controller assumes the car drives towards the right direction and increases the speed. When Kd is sufficiently small, the car's speed changes less than when Kd is large. In [this video](https://github.com/pechyony/pid-controller/blob/master/videos/no_s_D.mp4) we show the car driving with Kd set to zero while keeping all other parameter values. The car successfully completes several laps,  but has larger root mean squared cross-track error, 0.403, than the ove with the tuned value of Kd. Also the car had larger average speed, 36.41 mph, than the one with the tuned value of Kd. Overall, the metric value with Kd=0 was -4733.4, which is slightly higher than the one with the tuned Kd. We conclude that the parameter Kd of the speed controller is the least important parameter among the six parameters of two controllers.  






