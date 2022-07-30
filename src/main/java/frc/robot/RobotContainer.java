// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.commands.DriveCommand;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gyro;

//import frc.robot.subsystems.SwerveOdometry;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Create robot's shuffboard operator interface
 // public static final ShuffleboardOI shuffleboard = new ShuffleboardOI();

  // The robot's subsystems are defined here...
  public static final Gyro gyro = new Gyro();
  public static final Drivetrain drivetrain = new Drivetrain();
  //public static final SwerveOdometry odometry = new SwerveOdometry();
  

  /**
   * Initialise the container for the robot. Contains subsystems, OI devices, and
   * commands.
   */
  public static void init() {

    drivetrain.setDefaultCommand(new DriveCommand(drivetrain));
    
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private static void configureButtonBindings() {
   
    // SwerveModuleState state1 = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    // SwerveModuleState state2 = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
    // SwerveModuleState state3 = new SwerveModuleState(-0.5, Rotation2d.fromDegrees(0));
    // SwerveModuleState state4 = new SwerveModuleState(0.5, Rotation2d.fromDegrees(0));

    // OI.driverkStartButton.whenPressed(() ->drivetrain.updateModulesPID());   

    // Command angleSetPoint1 = new RunCommand(() -> drivetrain.setModuleStates(new SwerveModuleState[]{state1, state1, state1, state1}), drivetrain);
    // Command stopState = new InstantCommand(drivetrain::stopMotors,drivetrain);  
    //OI.driverkAButton.whenHeld(angleSetPoint1).whenReleased(stopState);

     //kBack button: reset gyro
    OI.zeroButton.whenPressed(() -> gyro.resetGyro());


       
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public static Command getAutonomousCommand() {
   return null;
}
}