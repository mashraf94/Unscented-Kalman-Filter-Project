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

### The part implemented is all executed in the UKF instance method `UKF::ProcessMeasurement()` using the data input in the `MeasurementPackage` object: `meas_package`.
The input indicates:
1. Timestamp
2. Raw Measurements from Sensor
3. Sensor Type (RADAR or LASER)

### 1. Initialization
### Initialize a UKF upon detection of a sensor measurement:
1. Initialize the UKF instant attributes
   1. Declare the Dimensions of the State Vector `n_x_ = 5;` 
   2. Declare the Dimensions of the Augmented State Vector `n_aug_ = 7;`
   3. Calculate the Sigma Points' Spread Parameter `lambda_ = 3 - n_aug_;`
   4. Calculate the Sigma Points' Weight Vector of Size `2*n_aug + 1` for each of the 15 sigma point:
         * First Weight: `weights_(0) = lambda_ / (lambda_ + n_aug_);`
         * All Remaining Weights: `weights_ = 1 / (2*(lambda_ + n_aug_));`

2. Initialize UKF Vectors and Matrices:
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

         P_ : Size (n_x_ x n_x_) --> (5 x 5)
         
              Px Var
               0.15     0     0     0     0
                      Py Var
                0      0.15   0     0     0
                            V Var
                0       0     50    0     0
                                 Yaw Var
                0       0     0     50    0
                                       YawDot Var
                0       0     0     0     50
                
   * Predicted Sigma Points Matrix `Xsig_pred_`: This matrix is initialized with zeros of size `(n_x_ x 2*n_aug_+1)` --> (5 x 15)
   
3. Initialize time to `MeasurementPackage::timestamp_` to calculate `delta_t` between both current and next measurement.
4. Set `is_initialized_` to `true` to start Prediction and Update Steps upon the next sensor measurement.

### 2. Prediction
***Regardless of the type of input sensor type `meas_package.sensor_type_`:RADAR and LIDAR - the algorithm executes the same prediction functions***

1. Calculate the Augmented State Vector `x_aug` and Augmented State Covariance `P_aug`.
2. Generate 15 Augmented Sigma Points `Xsig_aug` for Previous estimated State Vector. Size > `(n_aug_ x 2*n_aug_+1)`: (7 x 15)
3. Predict Sigma Points representing the Current State Vector `Xsig_pred_`. Size > `(n_x_ x 2*n_aug_+1)`: (5 x 15)
4. Use predicted sigma points to estimate state mean and covariance

### 3. Update
***LIDAR and RADAR sensor data are treated differently since LIDAR data are in CARTESIAN coordinates, while RADAR data are in POLAR coordinates***

### 4. RMSE and NIS
### 5. Results
