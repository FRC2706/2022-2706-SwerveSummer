// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.CheckError;
import frc.robot.config.Config;
import frc.robot.config.FConfig;

public class SwerveModuleFalcon {

    TalonFX driveFalcon;
    TalonFX steeringFalcon;
    CANCoder encoder;

    /**
     * Constructs a SwerveModule.
     */
    public SwerveModuleFalcon(int driveCanID, TalonFXInvertType driveInverted, boolean driveSensorPhase, 
                              int steeringCanID, TalonFXInvertType steeringInverted, boolean steeringSensorPhase,
                              int encoderCanID, double encoderOffset) {

        // Drive
        driveFalcon = new TalonFX(driveCanID);
                
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.slot0.kP = Config.drive_kP;
        driveConfiguration.slot0.kI = Config.drive_kI;
        driveConfiguration.slot0.kD = Config.drive_kD;
        driveConfiguration.slot0.kF = Config.drive_kFF;
        driveConfiguration.slot0.integralZone = Config.drive_kIZone;
        driveConfiguration.voltageCompSaturation = 12.0; //12.0 volts is the default for Mk4 (need to add to Config.java)
        driveConfiguration.supplyCurrLimit.currentLimit = 80.0; // 80 amps is the default for Mk4 drive (need to add to Config.java)
        driveConfiguration.supplyCurrLimit.enable = true;

        CheckError.ctre(driveFalcon.configAllSettings(driveConfiguration, FConfig.CAN_TIMEOUT_LONG), "Failed to configure drive Falcon 500 settings");
        CheckError.ctre(driveFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, FConfig.CAN_TIMEOUT_SHORT), "Failed to set drive Falcon 500 feedback sensor");
        CheckError.ctre(driveFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, FConfig.STATUS_FRAME_GENERAL_PERIOD_MS, FConfig.CAN_TIMEOUT_SHORT), "Failed to configure drive Falcon status frame period");

        driveFalcon.setInverted(TalonFXInvertType.Clockwise); // GRABBED FROM SDS CODE, MOVE TO Config.java
        driveFalcon.setSensorPhase(true); // GRABBED FROM SDS CODE, MOVE TO Config.java
        driveFalcon.setNeutralMode(NeutralMode.Brake); // Need a Config.DRIVE_NEUTRAL_MODE
        driveFalcon.enableVoltageCompensation(true); // Related to driveConfiguration.voltageCompSaturation
        
            
        // Steering
        steeringFalcon = new TalonFX(steeringCanID);
                
        TalonFXConfiguration steeringConfiguration = new TalonFXConfiguration();
        steeringConfiguration.slot0.kP = Config.steering_kP;
        steeringConfiguration.slot0.kI = Config.steering_kI;
        steeringConfiguration.slot0.kD = Config.steering_kD;
        steeringConfiguration.slot0.kF = Config.steering_kFF;
        steeringConfiguration.slot0.integralZone = Config.steering_kIZone;
        steeringConfiguration.voltageCompSaturation = 12.0; //12.0 volts is the default for Mk4 (need to add to Config.java)
        steeringConfiguration.supplyCurrLimit.currentLimit = 20.0; // 20 amps is the default for Mk4 steering (need to add to Config.java)
        steeringConfiguration.supplyCurrLimit.enable = true;

        CheckError.ctre(steeringFalcon.configAllSettings(steeringConfiguration, FConfig.CAN_TIMEOUT_LONG), "Failed to configure steering Falcon 500 settings");
        CheckError.ctre(steeringFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, FConfig.CAN_TIMEOUT_SHORT), "Failed to set steering Falcon 500 feedback sensor");
        CheckError.ctre(steeringFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, FConfig.STATUS_FRAME_GENERAL_PERIOD_MS, FConfig.CAN_TIMEOUT_SHORT), "Failed to configure steering Falcon status frame period"); // Reduce CAN status frame rates

        steeringFalcon.setSensorPhase(true); // GRABBED FROM SDS CODE, MOVE TO Config.java
        steeringFalcon.setInverted(TalonFXInvertType.CounterClockwise); // GRABBED FROM SDS CODE, MOVE TO Config.java
        steeringFalcon.setNeutralMode(NeutralMode.Brake); // Need a Config.steering_NEUTRAL_MODE
        steeringFalcon.enableVoltageCompensation(true); // Related to steeringConfiguration.voltageCompSaturation

        
        // Encoder
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();
        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.magnetOffsetDegrees = encoderOffset;
        encoderConfiguration.sensorDirection = direction == Direction.CLOCKWISE;

        encoder = new CANCoder(encoderCanID);
        CheckError.ctre(encoder.configAllSettings(encoderConfiguration), "Failed to configure CANCoder");
        CheckError.ctre(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, FConfig.CANCODER_STATUS_FRAME_SENSOR_DATA, FConfig.CAN_TIMEOUT_SHORT), "Failed to configure CANCoder update rate");
    
        updateSteeringFromCanCoder();
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocity(), getSteeringAngle());
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        Rotation2d measuredAngle = getSteeringAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, measuredAngle);
        double velocity = state.speedMetersPerSecond;
        Rotation2d angle = ContinousPIDSparkMax.calculate(state.angle, measuredAngle);
        
        driveFalcon.set(ControlMode.Velocity, velocity / FConfig.DRIVE_SENSOR_VEL_CONVERSION);
        steeringFalcon.set(ControlMode.Position, angle.getRadians() / FConfig.STEERING_SENSOR_POS_CONVERSION);
    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * 
     * @return meters per second of the wheel
     */
    public double getVelocity() {
        return driveFalcon.getSelectedSensorVelocity() * FConfig.DRIVE_SENSOR_VEL_CONVERSION;
    }

    /**
     * Returns the angle the wheel is pointing in a Rotation2d.
     * 
     * @return angle of the wheel as a Rotation2d
     */
    public Rotation2d getSteeringAngle() {
        return new Rotation2d(steeringFalcon.getSelectedSensorPosition() * FConfig.STEERING_SENSOR_POS_CONVERSION);
    }

    /**
     * Gets a reading from the Lamprey and updates the SparkMax encoder (interal NEO encoder).
     * This is specific to Swerge. Other methods need to be written for other hardware.
     */
    public void updateSteeringFromCanCoder() {
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        CheckError.ctre(driveFalcon.setSelectedSensorPosition(angle / FConfig.STEERING_SENSOR_POS_CONVERSION, 0, FConfig.CAN_TIMEOUT_SHORT), "Failed to set Falcon 500 encoder position");
    }

    /**
     * Standard stop motors method for every subsystem.
     */
    public void stopMotors() {
        driveFalcon.neutralOutput();
        steeringFalcon.neutralOutput();
    }

    private Direction direction = Direction.COUNTER_CLOCKWISE;
    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}