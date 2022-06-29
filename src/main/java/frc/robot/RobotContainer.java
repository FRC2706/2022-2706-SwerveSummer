// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ModuleAngleFromJoystick;
import frc.robot.config.Config;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        // DriveSubsystem.getInstance().setDefaultCommand(
        //         // The left stick controls translation of the robot.
        //         // Turning is controlled by the X axis of the right stick.
        //         new RunCommand(
        //                 () -> DriveSubsystem.getInstance().drive(
        //                         driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_Y),
        //                         driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_X),
        //                         driverStick.getRawAxis(Config.RIGHT_CONTROL_STICK_X),
        //                         true),
        //                 DriveSubsystem.getInstance()));

        // SINGLE MODULE CONTROL, REMOVE WHEN SWITCHING TO 4 MODULES
        DriveSubsystem.getInstance().setDefaultCommand(
                    new ModuleAngleFromJoystick(() -> driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_Y), 
                                                () -> driverStick.getRawAxis(Config.LEFT_CONTROL_STICK_X),
                                                DriveSubsystem.getInstance()));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and 
     * then calling passing it to a {@link JoystickButton}.
     */
    private void configureButtonBindings() {
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                Config.kMaxAutoSpeed,
                Config.kMaxAutoAcceleration)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(Config.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                Config.kPThetaController, 0, 0, Config.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                DriveSubsystem.getInstance()::getPose, // Functional interface to feed supplier
                Config.kDriveKinematics,

                // Position controllers
                new PIDController(Config.kPXController, 0, 0),
                new PIDController(Config.kPYController, 0, 0),
                thetaController,
                DriveSubsystem.getInstance()::setModuleStates,
                DriveSubsystem.getInstance());

        // Reset odometry to the starting pose of the trajectory.
        DriveSubsystem.getInstance().resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> DriveSubsystem.getInstance().drive(0, 0, 0, false));
    }
}
