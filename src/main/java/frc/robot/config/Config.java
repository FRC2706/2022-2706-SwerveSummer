// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Config {
    /* Drive Constants */
    public static final int CANID_FRONT_LEFT_DRIVE = 6;
    public static final int CANID_REAR_LEFT_DRIVE = 0;
    public static final int CANID_FRONT_RIGHT_DRIVE = 0;
    public static final int CANID_REAR_RIGHT_DRIVE = 0;

    public static final int CANID_FRONT_LEFT_STEERING = 5;
    public static final int CANID_REAR_LEFT_STEERING = 0;
    public static final int CANID_FRONT_RIGHT_STEERING = 0;
    public static final int CANID_REAR_RIGHT_STEERING = 0;

    public static final double turningEncoderConstant = (2*Math.PI)/8.0;
    public static final double drivetrainEncoderConstant = 0.1016*Math.PI*(1/(60*7.615));

    public static final boolean INVERTED_FRONT_LEFT_DRIVE = false;
    public static final boolean INVERTED_REAR_LEFT_DRIVE = false;
    public static final boolean INVERTED_FRONT_RIGHT_DRIVE = false;
    public static final boolean INVERTED_REAR_RIGHT_DRIVE = false;

    public static final boolean INVERTED_FRONT_LEFT_STEERING = false;
    public static final boolean INVERTED_REAR_LEFT_STEERING = false;
    public static final boolean INVERTED_FRONT_RIGHT_STEERING = false;
    public static final boolean INVERTED_REAR_RIGHT_STEERING = false;

    public static final int KLAMPREYCHANNEL_FRONT_LEFT= 7;
    public static final int KLAMPREYCHANNEL_REAR_LEFT = 0;
    public static final int KLAMPREYCHANNEL_FRONT_RIGHT = 0;
    public static final int KLAMPREYCHANNEL_REAR_RIGHT = 0;  
    
    public static final double drive_kIZone = 0.0;
    public static final double drive_kFF = 0.000153/drivetrainEncoderConstant; // These can also be module specific.
    public static final double drive_kP = 0.00009/drivetrainEncoderConstant; // Hopefully they won't need to be.
    public static final double drive_kI = 0.0; // Depends on hardware differences.
    public static final double drive_kD =0.0004/drivetrainEncoderConstant;

    public static final double steering_kFF = 0;
    public static final double steering_kP = 0.5;
    public static final double steering_kI = 0.0;
    public static final double steering_kD = 0.0;
    public static final double steering_kIZone = 0; //5 degrees
    
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.5;

    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.7;
    public static double kLampreyOffsetFL = -0.818;
    public static double kLampreyOffsetRL = 0.0;
    public static double kLampreyOffsetFR = 0.0;
    public static double kLampreyOffsetRR = 0.0;
    public static FluidConstant<Double> fluid_LampreyOffsetFL = new FluidConstant<>("Lamprey Offset Radians", kLampreyOffsetFL, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));
    
    public static FluidConstant<Double> fluid_LampreyOffsetRL = new FluidConstant<>("Lamprey Offset Radians", kLampreyOffsetRL, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

    public static FluidConstant<Double> fluid_LampreyOffsetFR = new FluidConstant<>("Lamprey Offset Radians", kLampreyOffsetFR, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

     public static FluidConstant<Double> fluid_LampreyOffsetRR = new FluidConstant<>("Lamprey Offset Radians", kLampreyOffsetRR, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));



    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),   
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static int CAN_PIGEON = 27;

    /* Module Constants */
    // The absolute max speed that a module can reach.
    public static final double kMaxAttainableWheelSpeed = 3.0;

    public static final double kWheelDiameterMeters = 0.1016;

    /* Joystick Constants */
    public static int LEFT_CONTROL_STICK_Y = 1;
    public static int LEFT_CONTROL_STICK_X = 0;

    public static int RIGHT_CONTROL_STICK_Y = 5;
    public static int RIGHT_CONTROL_STICK_X = 4;

    /** Auto Constants */
    public static final double kMaxAutoSpeed = 3; // m/s
    public static final double kMaxAutoAcceleration = 3; // m/s/s
    public static final double kMaxAutoAngularSpeed = Math.PI; // rad/s
    public static final double kMaxAutoAngularAcceleration = Math.PI; // rad/s/s

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAutoAngularSpeed, kMaxAutoAngularAcceleration);

}
