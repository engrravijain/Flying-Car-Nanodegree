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