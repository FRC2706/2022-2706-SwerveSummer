// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

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
public final class ConfigMerge {
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

    public static final TalonFXInvertType INVERTED_FRONT_LEFT_DRIVE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType INVERTED_REAR_LEFT_DRIVE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType INVERTED_FRONT_RIGHT_DRIVE = TalonFXInvertType.Clockwise;
    public static final TalonFXInvertType INVERTED_REAR_RIGHT_DRIVE = TalonFXInvertType.Clockwise;

    public static final TalonFXInvertType INVERTED_FRONT_LEFT_STEERING = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType INVERTED_REAR_LEFT_STEERING = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType INVERTED_FRONT_RIGHT_STEERING = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType INVERTED_REAR_RIGHT_STEERING = TalonFXInvertType.CounterClockwise;
	
	public static final boolean SENSOR_PHASE_FRONT_LEFT_DRIVE = true;
	public static final boolean SENSOR_PHASE_REAR_LEFT_DRIVE = true;
	public static final boolean SENSOR_PHASE_FRONT_RIGHT_DRIVE = true;
	public static final boolean SENSOR_PHASE_REAR_RIGHT_DRIVE = true;

	public static final boolean SENSOR_PHASE_FRONT_LEFT_STEERING = true;
	public static final boolean SENSOR_PHASE_REAR_LEFT_STEERING = true;
	public static final boolean SENSOR_PHASE_FRONT_RIGHT_STEERING = true;
	public static final boolean SENSOR_PHASE_REAR_RIGHT_STEERING = true;
	
    public static final int CANID_FRONT_LEFT_CANCODER = 12;
    public static final int CANID_REAR_LEFT_CANCODER = 9;
    public static final int CANID_FRONT_RIGHT_CANCODER = 11;
    public static final int CANID_REAR_RIGHT_CANCODER = 10;  
    
    public static final double drive_kIZone = 0.15;
    public static final double drive_kFF = 0.25; // These can also be module specific.
    public static final double drive_kP = 0.2; // Hopefully they won't need to be.
    public static final double drive_kI = 0.002; // Depends on hardware differences.
    public static final double drive_kD =1.0;
	public static FluidConstant<Double> fluid_drive_kFF = new FluidConstant<>("Drive kFF", drive_kFF, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

        public static FluidConstant<Double> fluid_drive_kP = new FluidConstant<>("Drive kP", drive_kP, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

	public static FluidConstant<Double> fluid_drive_kI = new FluidConstant<>("Drive kI", drive_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));


	public static FluidConstant<Double> fluid_drive_kD = new FluidConstant<>("Drive kD", drive_kD, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

	public static FluidConstant<Double> fluid_drive_kIZone = new FluidConstant<>("Drive kIZone", drive_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));
    // Distance between centers of right and left wheels on robot

    public static final double steering_kFF = 0;
    public static final double steering_kP = 0.8;
    public static final double steering_kI = 0.016;
    public static final double steering_kD = 1.6;
    public static final double steering_kIZone = 0.05; //5 degrees
    public static FluidConstant<Double> fluid_steering_kFF = new FluidConstant<>("Steering kFF", steering_kFF, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

    public static FluidConstant<Double> fluid_steering_kP = new FluidConstant<>("Steering kP", steering_kP, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

	public static FluidConstant<Double> fluid_steering_kI = new FluidConstant<>("Steering kI", steering_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

	public static FluidConstant<Double> fluid_steering_kD = new FluidConstant<>("Steering kD", steering_kD, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));
					
	public static FluidConstant<Double> fluid_steering_kIZone = new FluidConstant<>("Steering kIZone", steering_kI, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = 0.5;

    // Distance between front and back wheels on robot
    public static final double kWheelBase = 0.7;
    public static double kLampreyOffsetFL = 0.0;
    public static double kLampreyOffsetRL = 0.0;
    public static double kLampreyOffsetFR = 0.0;
    public static double kLampreyOffsetRR = 0.0;
    public static FluidConstant<Double> fluid_LampreyOffsetFL = new FluidConstant<>("Lamprey Offset Radians FL", kLampreyOffsetFL, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));
    
    public static FluidConstant<Double> fluid_LampreyOffsetRL = new FluidConstant<>("Lamprey Offset Radians RL", kLampreyOffsetRL, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

    public static FluidConstant<Double> fluid_LampreyOffsetFR = new FluidConstant<>("Lamprey Offset Radians FR", kLampreyOffsetFR, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

     public static FluidConstant<Double> fluid_LampreyOffsetRR = new FluidConstant<>("Lamprey Offset Radians RR", kLampreyOffsetRR, true)
                    .registerToTable(NetworkTableInstance.getDefault().getTable("SwerveModule"));

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

				/** Stuff added to handle Falcons */
    public static final int CAN_TIMEOUT_SHORT = 100;
    public static final int CAN_TIMEOUT_LONG = 250;
    public static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;
    public static final int CANCODER_STATUS_FRAME_SENSOR_DATA = 10;
    public static final double TICKS_PER_ROTATION = 2048.0;
    public static final double MK4_WHEEL_DIAMTER = 0.10033;

    private static final double DRIVE_GEAR_REDUCTION = (14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0); // THIS IS FOR THE MK4_L1
    private static final double DRIVE_SENSOR_POS_CONVERSION = Math.PI * MK4_WHEEL_DIAMTER * DRIVE_GEAR_REDUCTION / TICKS_PER_ROTATION;
    public static final double DRIVE_SENSOR_VEL_CONVERSION = DRIVE_SENSOR_POS_CONVERSION * 10.0;

    private static final double STEERING_GEAR_REDUCTION = (15.0 / 32.0) * (10.0 / 60.0); // THIS IS FOR THE MK4_L1
    public static final double STEERING_SENSOR_POS_CONVERSION = 2.0 * Math.PI / TICKS_PER_ROTATION * STEERING_GEAR_REDUCTION;

    public static final double FL_ENCODER_OFFSET = -(155 + 180);
    public static final double FR_ENCODER_OFFSET = -(94 + 180);
    public static final double RL_ENCODER_OFFSET = -(200 + 180);
    public static final double RR_ENCODER_OFFSET = -(135 + 180);

	
}
