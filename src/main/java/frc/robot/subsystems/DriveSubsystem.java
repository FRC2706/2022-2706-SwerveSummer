// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.Config;

public class DriveSubsystem extends SubsystemBase {
    // Instance for singleton class
    private static DriveSubsystem instance;

    // Robot swerve modules
    private final SwerveModuleFalcon m_frontLeft = new SwerveModuleFalcon(Config.CANID_FRONT_LEFT_DRIVE, Config.INVERTED_FRONT_LEFT_DRIVE, Config.SENSOR_PHASE_FRONT_LEFT_DRIVE, Config.CANID_FRONT_LEFT_STEERING, Config.INVERTED_FRONT_LEFT_STEERING, Config.SENSOR_PHASE_FRONT_LEFT_STEERING, Config.CANID_FRONT_LEFT_CANCODER, Config.FL_ENCODER_OFFSET, "FL");

    private final SwerveModuleFalcon m_rearLeft = new SwerveModuleFalcon(Config.CANID_REAR_LEFT_DRIVE, Config.INVERTED_REAR_LEFT_DRIVE, Config.SENSOR_PHASE_REAR_LEFT_DRIVE, Config.CANID_REAR_LEFT_STEERING, Config.INVERTED_REAR_LEFT_STEERING, Config.SENSOR_PHASE_REAR_LEFT_STEERING, Config.CANID_REAR_LEFT_CANCODER, Config.RL_ENCODER_OFFSET, "RL");

    private final SwerveModuleFalcon m_frontRight = new SwerveModuleFalcon(Config.CANID_FRONT_RIGHT_DRIVE, Config.INVERTED_FRONT_RIGHT_DRIVE, Config.SENSOR_PHASE_FRONT_RIGHT_DRIVE, Config.CANID_FRONT_RIGHT_STEERING, Config.INVERTED_FRONT_RIGHT_STEERING, Config.SENSOR_PHASE_FRONT_RIGHT_STEERING, Config.CANID_FRONT_RIGHT_CANCODER, Config.FR_ENCODER_OFFSET, "FR");

    private final SwerveModuleFalcon m_rearRight = new SwerveModuleFalcon(Config.CANID_REAR_RIGHT_DRIVE, Config.INVERTED_REAR_RIGHT_DRIVE, Config.SENSOR_PHASE_REAR_RIGHT_DRIVE, Config.CANID_REAR_RIGHT_STEERING, Config.INVERTED_REAR_RIGHT_STEERING, Config.SENSOR_PHASE_REAR_RIGHT_STEERING, Config.CANID_REAR_RIGHT_CANCODER, Config.RR_ENCODER_OFFSET, "RR");

    // The gyro sensor
    private final PigeonIMU m_pigeon = new PigeonIMU(Config.CAN_PIGEON);

    // Odometry class for tracking robot pose
    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(Config.kDriveKinematics, getHeadingRotation2d());

    /** Get instance of singleton class */
    public static DriveSubsystem getInstance() {
        if (instance == null)
            instance = new DriveSubsystem();
        return instance;
    }
    
    /** Creates a new DriveSubsystem. */
    private DriveSubsystem() {
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        m_odometry.update(
                getHeadingRotation2d(),
                m_frontLeft.getState(),
                m_frontRight.getState(),
                m_rearLeft.getState(),
                m_rearRight.getState());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        m_odometry.resetPosition(pose, getHeadingRotation2d());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = Config.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getHeadingRotation2d()));
        } else {
            swerveModuleStates = Config.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, rot));
        }
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Config.kMaxAttainableWheelSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
                desiredStates, Config.kMaxAttainableWheelSpeed);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return m_pigeon.getFusedHeading();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public Rotation2d getHeadingRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /**
     * Standard stop motors method for every subsystem.
     */
    public void stopMotors() {
        m_frontLeft.stopMotors();
        m_rearLeft.stopMotors();
        m_frontRight.stopMotors();
        m_rearRight.stopMotors();
    }

    public void updateModulesPID(){
        m_frontLeft.updatePIDValues();
    }

    public void resetEncodersFromCanCoder() {
        m_frontLeft.updateSteeringFromCanCoder();
        m_frontRight.updateSteeringFromCanCoder();
        m_rearLeft.updateSteeringFromCanCoder();
        m_rearRight.updateSteeringFromCanCoder();
    }

}
