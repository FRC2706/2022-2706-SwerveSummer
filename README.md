# 2022-2706-SwerveSummer
Doing swerve in the summer.

This codebase originated from the WPILib example code avaible here: 
https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/swervecontrollercommand

We removed the PWM motor controlers in the WPILib codebase and replaced them with CAN motor controllers.

Currently supports SDS mk4.

Also supports Swerge V4 with SwerveModuleSparkMax however the DriveSubsystem needs changes to use SwerveModuleSparkMax.


# Going to Kingston to test code
Thank you to team 7480 Machine Mavericks once again.

## Teleop
After tuning the PIDF and IZone values for the drive and steering motors, kinematics worked great to control the robot in teleop.

Video of teleop control:
https://www.youtube.com/watch?v=8znFHGSqZx4

## Autonomous
Once Kinematics is tuned with a PIDF for velocity control and PID for steering, autonomous came shortly after.

Autonomous uses the SwerveControllerCommand provided by WPILib:
https://github.com/wpilibsuite/allwpilib/blob/main/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommand.java

This command uses the Trajectory objects to control the X and Y of the robot. It also has different constructors to state what the heading should do while it goes through the X and Y of the trajectory.

SwerveCommandMerge is something we created to make it easier to construct SwerveControllerCommands. SwerveCommandMerge only needs to be given a trajectory and what the heading should do instead of the 7 parameter long constructor that SwerveControllerCommand needs.

### Autonomous test1 - Slow Speed

Video of slow speed:
https://youtu.be/DfByJQQnnsY

First thing to note. We disabled the SwerveControllerCommand from controlling the rotation so what's seen in the tests is the heading drifting and not correcting itself. We had issues with the rotation and not enough time to investiage them, so the video is only demonstrating the X and Y control.

Path planned:
![PathShownInVideo](https://user-images.githubusercontent.com/43829793/182493698-11f79e3f-ed2b-495a-8bd7-a9de1e87a0d6.jpg)

Shuffle board reporting the error in the X and Y of the path. The veritcal axis is in meters.
![AutoPath-SlowSpeed-Shuffleboard](https://user-images.githubusercontent.com/43829793/182493753-8ac7358e-48c3-41bd-a84d-200af5c03d65.jpg)


### Autonomous test2 - Fast Speed

Same path as slow speed.

Video of fast speed:
https://youtu.be/vVuHyKpA6Yo

Shuffle board reporting the error in the X and Y of the path. The veritcal axis is in meters.
![AutoPath-FastSpeed-Shuffleboard](https://user-images.githubusercontent.com/43829793/182494653-6e07800b-4c19-4647-9af0-05dc9af683bf.jpg)


# Instructions

## Part 1 - June 6th
Started with a 20 minute presentation about what is Swerve.<br />
https://docs.google.com/presentation/d/1UlFns79VPLa6JWTARuPIhunhoWoVWEZtnUoII4OY9Xs/edit?usp=sharing

The last 2 slides have some instructions which are copied below:<br />
1. Clone SwerveSummer2022 and make a branch with your name as the title. Your branch should be from the NewStudentsStartHere branch.
2. Setup 2 CANSparkMax objects in SwerveModule.java<br />
           -  Set PIDF+IZone to 0 for now. We will tune them on another day.
4. Complete setDesiredState method and getSteeringAngle method in SwerveModule.java

Check out this PDF for a guide to which CANSparkMax settings need to be set.<br />
https://drive.google.com/file/d/1IVFa-3UEYdORYtp_CFTkWB-YxjUFXocq/view?usp=sharing
