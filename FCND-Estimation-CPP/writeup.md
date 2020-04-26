# Estimation Project

Welcome to the estimation project. In this project, you will be developing the estimation portion of the controller used in the CPP simulator. By the end of the project, your simulated quad will be flying with your estimator and your custom controller (from the previous project)!

## Step 1: Sensor Noise

After choosing scenario 06_NoisySensors, I run the simulation for once to collect noisy sensor data and used logs from config/log/Graph1.txt (GPS X data) and config/log/Graph2.txt (Accelerometer X data) files to calculate standard deviations.

I used [sensor_noise.py](./sensor_noise.py) to process the logged files to figure out the standard deviation of the the GPS X signal and the IMU Accelerometer X signal.

The Values of standard deviation calculated were plugged into [06_SensorNoise.txt](./config/06_SensorNoise.txt). 

`MeasuredStdDev_GPSPosXY`: 0.7

`MeasuredStdDev_AccelXY`: 0.49

### Result

```
Simulation #3 (../config/06_SensorNoise.txt)
PASS: ABS(Quad.GPS.X-Quad.Pos.X) was less than MeasuredStdDev_GPSPosXY for 67% of the time
PASS: ABS(Quad.IMU.AX-0.000000) was less than MeasuredStdDev_AccelXY for 67% of the time
```

## Step 2: Attitude Estimation

I modified the `UpdateFromIMU()` function in [QuadEstimatorEKF.cpp](./src/QuadEstimatorEKF.cpp) to modify linear implementation of the filter to a non-linear one to get better results. The success criteria is Your attitude estimator needs to get within 0.1 rad for each of the Euler angles for at least 3 seconds.

### Result

```
Simulation #3 (../config/07_AttitudeEstimation.txt)
PASS: ABS(Quad.Est.E.MaxEuler) was less than 0.100000 for at least 3.000000 seconds
```

## Step 3: Prediction Step

### Scenario 8

This scenario had no specific measurable criteria being checked. However I completed the task by implementing the prediction step filter in `PredictState()` function in [QuadEstimatorEKF.cpp](./src/QuadEstimatorEKF.cpp).

The estimator state tracks the actual state with only reasonably slow drift.

### Scenario 9

Made required code changes in `GetRbgPrime()` and `Predict()` functions in [QuadEstimatorEKF.cpp](./src/QuadEstimatorEKF.cpp).


In order to capture the magnitude of error I tuned the parameters in [QuadEstimatorEKF.txt](./config/QuadEstimatorEKF.txt).

This scenario had no specific measurable criteria being checked

## Step 4: Magnetometer Update

Following Section 7.3.2 of [Estimation for Quadrotors](https://www.overleaf.com/read/vymfngphcccj#/54894644/) the equations for `zFromX` and `hPrime` for Magnetometer is implemented as follows in `UpdateFromMag()` function in [QuadEstimatorEKF.cpp](./src/QuadEstimatorEKF.cpp).

```
float estYaw = ekfState(6);
if (estYaw - magYaw > +F_PI) estYaw -= 2.f*F_PI;
if (estYaw - magYaw < -F_PI) estYaw += 2.f*F_PI;

zFromX(0) = estYaw;
hPrime(0, 6) = 1;
```

Tuning the parameter `QYawStd` to .1 in [QuadEstimatorEKF.txt](./config/QuadEstimatorEKF.txt) approximately captures the magnitude of the drift.

### Result

```
Simulation #5 (../config/10_MagUpdate.txt)
PASS: ABS(Quad.Est.E.Yaw) was less than 0.120000 for at least 10.000000 seconds
PASS: ABS(Quad.Est.E.Yaw-0.000000) was less than Quad.Est.S.Yaw for 71% of the time
```

## Step 5: Closed Loop + GPS Update

Firstly I ran the scenario `11_GPSUpdate` which At the moment using both an ideal estimator and and ideal IMU & observed position and velocity errors (bottom right). This is because the GPS is not implemented yet. 

I updated the `UpdateFromGPS()` function in [QuadEstimatorEKF.txt](./config/QuadEstimatorEKF.txt). 

### Result

```
Simulation #3 (../config/11_GPSUpdate.txt)
PASS: ABS(Quad.Est.E.Pos) was less than 1.000000 for at least 20.000000 seconds
```

