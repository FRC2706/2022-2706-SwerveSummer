// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.auto.SwerveCommandMerge;
import frc.robot.commands.ModuleAngleFromJoystick;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.ResetOdometry;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The driver's controller
    Joystick driverStick = new Joystick(0);
    Joystick controlStick = new Joystick(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();


        //Configure default commands
        DriveSubsystem.getInstance().setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> DriveSubsystem.getInstance().drive(
                                driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_Y) * Config.kMaxAttainableWheelSpeed,
                                driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_X) * Config.kMaxAttainableWheelSpeed,
                                driverStick.getRawAxis(Config.RIGHT_CONTROL_STICK_X) * Config.kMaxAutoAngularSpeed,
                                true),
                        DriveSubsystem.getInstance()));

        //SINGLE MODULE CONTROL, REMOVE WHEN SWITCHING TO 4 MODULES
        //DriveSubsystem.getInstance().setDefaultCommand(
                    //new ModuleAngleFromJoystick(() -> driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_Y), 
                                                //() -> driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_X),
                                                //DriveSubsystem.getInstance()));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and 
     * then calling passing it to a {@link JoystickButton}.
     */


    private void configureButtonBindings() {
        SwerveModuleState state1 = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        SwerveModuleState state2 = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
        SwerveModuleState state3 = new SwerveModuleState(-0.5, Rotation2d.fromDegrees(0));
        SwerveModuleState state4 = new SwerveModuleState(0.5, Rotation2d.fromDegrees(0));

        Command updateModulesPID = new InstantCommand(DriveSubsystem.getInstance()::updateModulesPID, DriveSubsystem.getInstance());
        new JoystickButton(driverStick, XboxController.Button.kStart.value).whenPressed(updateModulesPID);

        Command angleSetPoint1 = new RunCommand(() -> DriveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state1, state1, state1, state1}), DriveSubsystem.getInstance());
        new JoystickButton(driverStick, XboxController.Button.kA.value).whenHeld(angleSetPoint1).whenReleased(new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance()));
        
        Command angleSetPoint2 = new RunCommand(() -> DriveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state2, state1, state1, state1}), DriveSubsystem.getInstance());
        new JoystickButton(driverStick, XboxController.Button.kB.value).whenHeld(angleSetPoint2).whenReleased(new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance()));

        Command angleSetPoint3 = new RunCommand(() -> DriveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state2, state2, state2, state2}), DriveSubsystem.getInstance());
        new JoystickButton(driverStick, XboxController.Button.kLeftBumper.value).whenHeld(angleSetPoint3).whenReleased(new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance()));
        
        Command speedSetPoint1 = new RunCommand(() -> DriveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state3, state3, state3, state3}), DriveSubsystem.getInstance());
        new JoystickButton(driverStick, XboxController.Button.kX.value).whenHeld(speedSetPoint1).whenReleased(new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance()));
        
        Command speedSetPoint2 = new RunCommand(() -> DriveSubsystem.getInstance().setModuleStates(new SwerveModuleState[]{state4, state4, state4, state4}), DriveSubsystem.getInstance());
        new JoystickButton(driverStick, XboxController.Button.kY.value).whenHeld(speedSetPoint2).whenReleased(new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance()));

        new JoystickButton(driverStick, XboxController.Button.kBack.value).whenHeld(new ResetGyro());  

        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        int selectorIndex = 0;
        if(selectorIndex == 0){
            return null;
        }
        else if(selectorIndex == 1){
			return new SequentialCommandGroup(
				new ResetOdometry(Robot.trajStraightForwardPath.getInitialPose()),
				new SwerveCommandMerge(Robot.trajStraightForwardPath),
				new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance())
			);
        }
        else if(selectorIndex == 2){
			return new SequentialCommandGroup(
				new ResetOdometry(Robot.trajArcPath.getInitialPose()),
				new SwerveCommandMerge(Robot.trajArcPath),
				new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance())
			);  
        }
        else if(selectorIndex == 3){
			return new SequentialCommandGroup(
				new ResetOdometry(Robot.trajSCurve.getInitialPose()),
				new SwerveCommandMerge(Robot.trajSCurve),
				new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance())
			);  
        }
        else if(selectorIndex == 4){
			return new SequentialCommandGroup(
				new ResetOdometry(Robot.trajLongPath.getInitialPose()),
				new SwerveCommandMerge(Robot.trajLongPath),
				new InstantCommand(DriveSubsystem.getInstance()::stopMotors, DriveSubsystem.getInstance())
			); 
        }
		return null;
    }
}
