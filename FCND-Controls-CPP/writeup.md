# FCND-Controls-CPP Project Write-up

Below I'll explain all the The steps followed to pass all scenarios for the project.

## Intro (scenario 1)

By setting the mass of quad to `0.5` in (QuadControlParams.txt)[./config/QuadControlParams.txt] will make the quad more or less stay in the same spot.

### Result

<img src="animations/scenario1.gif" width="500" height="500" alt="Scenario 1 "/>
<br><br>

```
Simulation #1 (../config/1_Intro.txt)
PASS: ABS(Quad.PosFollowErr) was less than 0.500000 for at least 0.800000 seconds
```

## Body rate and roll/pitch control (scenario 2)

First, you will implement the body rate and roll / pitch control.  For the simulation, you will use `Scenario 2`.  In this scenario, you will see a quad above the origin.  It is created with a small initial rotation speed about its roll axis.  Your controller will need to stabilize the rotational motion and bring the vehicle back to level attitude.

To accomplish this, you will:

1. Implement body rate control

 - implement the code in the function `GenerateMotorCommands()`
 
   the function needs to be resolved using the below equations, Where all the F_1 to F_4 are the motor's thrust
   <p align="center">
    <img src="images/moments_force_eq.gif" width="500"/>
   </p>
   
   let `l` denote the perpendicular distance of motors from the axes.
   ```
   float l = L / (sqrt(2));
   ```
   I calculated the thrust required on each motor using the below equations.
   ```
   float t1 = collThrustCmd / 4;
   float t2 = momentCmd.z / (kappa * 4);
   float t3 = momentCmd.x / (4 * l);
   float t4 = momentCmd.y / (4 * l);

   //  Motor1          Motor2
   //            Q
   //  Motor3          Motor4

   cmd.desiredThrustsN[0] = t1 - t2 + t3 + t4; // Motor1
   cmd.desiredThrustsN[1] = t1 + t2 - t3 + t4; // Motor2
   cmd.desiredThrustsN[2] = t1 + t2 + t3 - t4; // Motor3
   cmd.desiredThrustsN[3] = t1 - t2 - t3 - t4; // Motor4
   ```
 - implement the code in the function `BodyRateControl()`
   
   ```
   V3F I;
   I.x = Ixx;
   I.y = Iyy;
   I.z = Izz;
  
   momentCmd = I * kpPQR * (pqrCmd - pqr);
   ```
 - Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot
   
   Tuned `kpPQR` to `82, 92, 15` 

2. Implement roll / pitch control
We won't be worrying about yaw just yet.

 - implement the code in the function `RollPitchControl()`
 
 ```
if (collThrustCmd > 0) {
	  float c = -collThrustCmd / mass;

	  float b_x_c = CONSTRAIN((accelCmd.x / c), -maxTiltAngle, maxTiltAngle);
	  float b_x_err = b_x_c - R(0, 2);
      float b_x_p_term = kpBank * b_x_err;

	  float b_y_c = CONSTRAIN((accelCmd.y / c), -maxTiltAngle, maxTiltAngle);
	  float b_y_err = b_y_c - R(1, 2);
	  float b_y_p_term = kpBank * b_y_err;

	  pqrCmd.x = (R(1, 0)*b_x_p_term - R(0, 0)*b_y_p_term) / R(2, 2);
	  pqrCmd.y = (R(1, 1)*b_x_p_term - R(0, 1)*b_y_p_term) / R(2, 2);
  }
  else {
	  pqrCmd.x = 0;
	  pqrCmd.y = 0;
  }

  pqrCmd.z = 0;
```

tuned `kpBank` to `12` to minimize settling time but avoid too much overshoot

If successful you should now see the quad level itself (as shown below), though it’ll still be flying away slowly since we’re not controlling velocity/position!  You should also see the vehicle angle (Roll) get controlled to 0.

### Result

<p align="center">
<img src="animations/scenario2.gif" width="500" alt="scenario 2"/>
</p>

```
Simulation #1 (../config/2_AttitudeControl.txt)
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

## Position/velocity and yaw angle control (scenario 3)

There are 2 identical quads in this scenario, one offset from its target point initialized with zero yaw and second offset from target point with 45 deg yaw. The goal is to stabilize both quads and make them reach their targeted lateral position, while maintaining their altitude.

The code can be found in `LateralPositionControl()`, `AltitudeControl()` and `YawControl()` functions. 

Tuned `kpPosXY` `kpPosZ` `KiPosZ` `kpVelXY` `kpVelZ` to achieve the expected results.

### Result

<p align="center">
<img src="animations/scenario3.gif" width="500" alt="scenario 3"/>
</p>

```
Simulation #1 (../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
```

## Non-idealities and robustness (scenario 4)

In this scenario, there are 3 quadrotors with some non-idealities:

- The green quad has its center of mass shifted back
- The orange vehicle is an ideal quad
- The red vehicle is heavier than usual

The main task is to relax the controller to improve the robustness of the control system and get all the quads to reach their destination. Here I modified the `AltitudeControl()` to add basic integral control to help with the different-mass vehicle.

### Result

<p align="center">
<img src="animations/scenario4.gif" width="500" alt="scenario 4"/>
</p>

```
Simulation #1 (../config/4_Nonidealities.txt)
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
```

## Tracking trajectories (scenario 5)

The drone (as tuned above) is able to follow the trajectory seamlessly:

### Result

<p align="center">
<img src="animations/scenario5.gif" width="500" alt="scenario 5"/>
</p>

```
Simulation #1 (../config/5_TrajectoryFollow.txt)
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```