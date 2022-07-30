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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class SwerveModuleFalcon {

    TalonFX driveFalcon;
    TalonFX steeringFalcon;
    CANCoder encoder;
    private NetworkTable swerveModuleTable;
    private NetworkTableEntry desiredSpeedEntry;
    private NetworkTableEntry desiredAngleEntry;
    private NetworkTableEntry currentSpeedEntry;
    private NetworkTableEntry currentAngleEntry;
    private NetworkTableEntry speedError;
    private NetworkTableEntry angleError;
    private NetworkTableEntry currentCanCoderEntry;
    /**
     * Constructs a SwerveModule.
     */
    public SwerveModuleFalcon(int driveCanID, TalonFXInvertType driveInverted, boolean driveSensorPhase, 
                              int steeringCanID, TalonFXInvertType steeringInverted, boolean steeringSensorPhase,
                              int encoderCanID, double encoderOffset, String ModuleName) {

        // Drive
        driveFalcon = new TalonFX(driveCanID);
                
        TalonFXConfiguration driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.slot0.kP = Config.fluid_drive_kP.get();
        driveConfiguration.slot0.kI = Config.fluid_drive_kI.get();
        driveConfiguration.slot0.kD = Config.fluid_drive_kD.get();
        driveConfiguration.slot0.kF = Config.fluid_drive_kFF.get();
        driveConfiguration.slot0.integralZone = Config.fluid_drive_kIZone.get();
        driveConfiguration.voltageCompSaturation = 12.0; //12.0 volts is the default for Mk4 (need to add to Config.java)
        driveConfiguration.supplyCurrLimit.currentLimit = 80.0; // 80 amps is the default for Mk4 drive (need to add to Config.java)
        driveConfiguration.supplyCurrLimit.enable = true;

        CheckError.ctre(driveFalcon.configAllSettings(driveConfiguration, Config.CAN_TIMEOUT_LONG), "Failed to configure drive Falcon 500 settings");
        CheckError.ctre(driveFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Config.CAN_TIMEOUT_SHORT), "Failed to set drive Falcon 500 feedback sensor");
        //CheckError.ctre(driveFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Config.STATUS_FRAME_GENERAL_PERIOD_MS, Config.CAN_TIMEOUT_SHORT), "Failed to configure drive Falcon status frame period");

        driveFalcon.setInverted(driveInverted); // GRABBED FROM SDS CODE, MOVE TO Config.java
        driveFalcon.setSensorPhase(driveSensorPhase); // GRABBED FROM SDS CODE, MOVE TO Config.java
        driveFalcon.setNeutralMode(Config.DRIVE_NEUTRAL_MODE); // Need a Config.DRIVE_NEUTRAL_MODE
        driveFalcon.enableVoltageCompensation(true); // Related to driveConfiguration.voltageCompSaturation
        
            
        // Steering
        steeringFalcon = new TalonFX(steeringCanID);
                
        TalonFXConfiguration steeringConfiguration = new TalonFXConfiguration();
        steeringConfiguration.slot0.kP = Config.fluid_steering_kP.get();
        steeringConfiguration.slot0.kI = Config.fluid_steering_kI.get();
        steeringConfiguration.slot0.kD = Config.fluid_steering_kD.get();
        steeringConfiguration.slot0.kF = Config.fluid_steering_kFF.get();
        steeringConfiguration.slot0.integralZone = Config.fluid_steering_kIZone.get();
        steeringConfiguration.voltageCompSaturation = 12.0; //12.0 volts is the default for Mk4 (need to add to Config.java)
        steeringConfiguration.supplyCurrLimit.currentLimit = 20.0; // 20 amps is the default for Mk4 steering (need to add to Config.java)
        steeringConfiguration.supplyCurrLimit.enable = true;

        CheckError.ctre(steeringFalcon.configAllSettings(steeringConfiguration, Config.CAN_TIMEOUT_LONG), "Failed to configure steering Falcon 500 settings");
        CheckError.ctre(steeringFalcon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, Config.CAN_TIMEOUT_SHORT), "Failed to set steering Falcon 500 feedback sensor");
        //CheckError.ctre(steeringFalcon.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Config.STATUS_FRAME_GENERAL_PERIOD_MS, Config.CAN_TIMEOUT_SHORT), "Failed to configure steering Falcon status frame period"); // Reduce CAN status frame rates

        steeringFalcon.setSensorPhase(steeringSensorPhase); // GRABBED FROM SDS CODE, MOVE TO Config.java
        steeringFalcon.setInverted(steeringInverted); // GRABBED FROM SDS CODE, MOVE TO Config.java
        steeringFalcon.setNeutralMode(Config.STEERING_NEUTRAL_MODE); // Need a Config.steering_NEUTRAL_MODE
        steeringFalcon.enableVoltageCompensation(true); // Related to steeringConfiguration.voltageCompSaturation

        
        // Encoder
        CANCoderConfiguration encoderConfiguration = new CANCoderConfiguration();
        encoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        encoderConfiguration.magnetOffsetDegrees = encoderOffset;
        encoderConfiguration.sensorDirection = direction == Direction.CLOCKWISE;

        encoder = new CANCoder(encoderCanID);
        CheckError.ctre(encoder.configAllSettings(encoderConfiguration), "Failed to configure CANCoder");
        CheckError.ctre(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, Config.CANCODER_STATUS_FRAME_SENSOR_DATA, Config.CAN_TIMEOUT_SHORT), "Failed to configure CANCoder update rate");

        String tableName = "Swerve Chassis/SwerveModule" + ModuleName;
        swerveModuleTable = NetworkTableInstance.getDefault().getTable(tableName);

        desiredSpeedEntry = swerveModuleTable.getEntry("Desired speed (m/s)");
        desiredAngleEntry = swerveModuleTable.getEntry("Desired angle (degrees)");
        currentSpeedEntry = swerveModuleTable.getEntry("Current speed (m/s)");
        currentAngleEntry = swerveModuleTable.getEntry("Current angle (degrees)");
        speedError = swerveModuleTable.getEntry("speed Error");
        angleError = swerveModuleTable.getEntry("angle Error");
        currentCanCoderEntry = swerveModuleTable.getEntry("CanCoder measurement");

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
    /*public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the reference state to avoid spinning further than 90 degrees
        double measuredVelocity = getVelocity();
        Rotation2d measuredAngle = getSteeringAngle();
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, measuredAngle);
        double velocity = state.speedMetersPerSecond;
        Rotation2d angle = ContinousPIDSparkMax.calculate(state.angle, measuredAngle);
        
        driveFalcon.set(ControlMode.Velocity, velocity / Config.DRIVE_SENSOR_VEL_CONVERSION);
        steeringFalcon.set(ControlMode.Position, angle.getRadians() / Config.STEERING_SENSOR_POS_CONVERSION);

        desiredSpeedEntry.setDouble(velocity);
        desiredAngleEntry.setDouble(angle.getDegrees());
        currentSpeedEntry.setDouble(measuredVelocity);
        currentAngleEntry.setDouble(measuredAngle.getDegrees());
        speedError.setDouble(velocity - measuredVelocity);
        angleError.setDouble(angle.getDegrees() - measuredAngle.getDegrees());
    }*/

    public void setDesiredState(SwerveModuleState desiredState) {
        //get the measure values from the embedded sensors
        double measuredVelocity = getVelocity();
        Rotation2d measuredAngle = getSteeringAngle();
        double deltaAngle = desiredState.angle.getRadians() - measuredAngle.getRadians();
        //make sure deltaAngle in [0,2pi]
        deltaAngle %= 2.0*Math.PI;
        if (deltaAngle < 0.0)
        deltaAngle += 2.0*Math.PI;
        //make sure detlaAngle in [-pi, pi]
        if ( deltaAngle > Math.PI && deltaAngle <= 2*Math.PI)
        {
            deltaAngle -= 2*Math.PI;
        }
        //make sure (desiredState.angle - currentAngle) difference is [-pi, pi], which is what optimize() requires.
        Rotation2d currentAngle = new Rotation2d(- deltaAngle + desiredState.angle.getRadians() );
        //Optimize the reference state to avoid spinning further than 90 degrees
        SwerveModuleState updatedDesiredstate = SwerveModuleState.optimize(desiredState, currentAngle);
        double velocity = updatedDesiredstate.speedMetersPerSecond;
        //deltaAngle now is in [-pi/2, pi/2], which is the angle to be adjusted.
        deltaAngle = updatedDesiredstate.angle.minus(currentAngle).getRadians();
        //todo:
       //-- Rotation2d angle = ContinousPIDSparkMax.calculate(state.angle, measuredAngle);
        driveFalcon.set(ControlMode.Velocity, velocity / Config.DRIVE_SENSOR_VEL_CONVERSION);
        steeringFalcon.set(ControlMode.Position, (measuredAngle.getRadians() + deltaAngle)/ Config.STEERING_SENSOR_POS_CONVERSION);
        desiredSpeedEntry.setDouble(velocity);
        desiredAngleEntry.setDouble(updatedDesiredstate.angle.getDegrees());
        currentSpeedEntry.setDouble(measuredVelocity);
        currentAngleEntry.setDouble(measuredAngle.getDegrees());
        speedError.setDouble(velocity - measuredVelocity);
        angleError.setDouble(updatedDesiredstate.angle.getDegrees() - measuredAngle.getDegrees());
    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * 
     * @return meters per second of the wheel
     */
    public double getVelocity() {
        return driveFalcon.getSelectedSensorVelocity() * Config.DRIVE_SENSOR_VEL_CONVERSION;
    }

    /**
     * Returns the angle the wheel is pointing in a Rotation2d.
     * 
     * @return angle of the wheel as a Rotation2d
     */
    public Rotation2d getSteeringAngle() {
        return new Rotation2d(steeringFalcon.getSelectedSensorPosition() * Config.STEERING_SENSOR_POS_CONVERSION);
    }

    /**
     * Gets a reading from the Lamprey and updates the SparkMax encoder (interal NEO encoder).
     * This is specific to Swerge. Other methods need to be written for other hardware.
     */
    public void updateSteeringFromCanCoder() {
        double angle = Math.toRadians(encoder.getAbsolutePosition());
        steeringFalcon.setSelectedSensorPosition(angle / Config.STEERING_SENSOR_POS_CONVERSION, 0, Config.CAN_TIMEOUT_SHORT);
        currentCanCoderEntry.setDouble(angle);
    }

    public void updatePIDValues(){
        driveFalcon.config_kP(0, Config.fluid_drive_kP.get(), Config.CAN_TIMEOUT_SHORT);
        driveFalcon.config_kI(0, Config.fluid_drive_kI.get(), Config.CAN_TIMEOUT_SHORT);
        driveFalcon.config_kD(0, Config.fluid_drive_kD.get(), Config.CAN_TIMEOUT_SHORT);
        driveFalcon.config_kF(0, Config.fluid_drive_kFF.get(), Config.CAN_TIMEOUT_SHORT);
        driveFalcon.config_IntegralZone(0, Config.fluid_drive_kIZone.get(), Config.CAN_TIMEOUT_SHORT);

        steeringFalcon.config_kP(0, Config.fluid_steering_kP.get(), Config.CAN_TIMEOUT_SHORT);
        steeringFalcon.config_kI(0, Config.fluid_steering_kI.get(), Config.CAN_TIMEOUT_SHORT);
        steeringFalcon.config_kD(0, Config.fluid_steering_kD.get(), Config.CAN_TIMEOUT_SHORT);
        steeringFalcon.config_kF(0, Config.fluid_steering_kFF.get(), Config.CAN_TIMEOUT_SHORT);
        steeringFalcon.config_IntegralZone(0, Config.fluid_steering_kIZone.get(), Config.CAN_TIMEOUT_SHORT);
        
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