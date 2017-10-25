# Unscented-Kalman-Filter-Project

In this project an Unscented Kalman Filter is implemented in C++ to track a moving object (bicycle), while assuming a *"Constant Turn Rate & Velocity Magnitude" (CTRV)* Model, fusing both LIDAR and RADAR sensor data to accurately estimate the object's position.

This project involves the [Udacity's 2-D Simulator](https://github.com/udacity/self-driving-car-sim/releases) which is connected to the UKF via an open source package called uWebSocketIO. This package facilitates the connection between the simulator and the code executed. The package does this by setting up a web socket server connection from the C++ program to the simulator, which acts as the host.

This repository includes two files that can be used to set up and intall [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./UnscentedKF

Here is the main protcol that main.cpp uses for uWebSocketIO in communicating with the simulator:

* INPUT: values provided by the simulator to the c++ program
    * ["sensor_measurement"] => the measurment that the simulator observed (either lidar or radar)

* OUTPUT: values provided by the c++ program to the simulator
    * ["estimate_x"] <= kalman filter estimated position x
    * ["estimate_y"] <= kalman filter estimated position y
    * ["rmse_x"]
    * ["rmse_y"]
    * ["rmse_vx"]
    * ["rmse_vy"]

---

## Other Important Dependencies
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

## Basic Build Instructions
1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./UnscentedKF`

---

### 1. Initialization
### Initialize a UKF upon detection of a sensor measurement:
1. Initialize UKF Vectors and Matrices:
   * State Vector x_: [Px, Py, v, yaw, yawdot]
        1. LIDAR Measurement: [Px, Py]
            * X Position = `Px;`
            * Y Position = `Py;`
            * Initialize Velocity v, Yaw Angle yaw, Turn Rate yawdot to `0`.
        
        2. RADAR Measurement: [rho, phi, rhodot]
           * X Position = `rho * cos(phi);`
           * Y Position = `rho * sin(phi);`
           * Initialize Velocity v, Yaw Angle yaw, Turn Rate yawdot to `0`.
           * Calculations based on the following visualization of the RADAR Measurements.
            <p align="center">
            <img align="center" src="./results/radar.png" alt="alt text" width=500 height=300>
            </p>
   
   * State Covariance Matrix P_:
      * Initialization for P_ is tuned manually for both LIDAR and RADAR measurements and the matrix below acquired the lowest RMSE results:

         P_:

               0.15     0     0     0     0

                0      0.15   0     0     0

                0       0     50    0     0

                0       0     0     50    0

                0       0     0     0     50

       
               

   

### 2. Prediction
### 3. Update
### 4. RMSE and NIS
### 5. Results
