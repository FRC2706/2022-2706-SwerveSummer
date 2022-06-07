// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    // CODE: Prepare 2 variables for both SparkMaxs, use the object called CANSparkMax

    /**
     * Constructs a SwerveModule.
     */
    public SwerveModule(/** CODE: Add parameters here for CONSTANTS that are specific to each module */) {

        // CODE: Construct both CANSparkMax objects and set all the nessecary settings (get CONSTANTS from Config or from the parameters of the constructor)

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
        
        
        // CODE: Pass the velocity (which is in meters per second) to velocity PID on drive SparkMax. (VelocityConversionFactor set so SparkMax wants m/s)

        // CODE: Pass the angle (which is in radians) to position PID on steering SparkMax. (PositionConversionFactor set so SparkMax wants radians)

    }

    /**
     * Returns the velocity of the wheel in meters per second.
     * 
     * @return meters per second of the wheel
     */
    public double getVelocity() {

        // CODE: Read encoder velocity from drive SparkMax and return m/s. (VelocityConversionFactor set so SparkMax returns m/s))

        return 0.0;
    }

    /**
     * Returns the angle the wheel is pointing in a Rotation2d.
     * 
     * @return angle of the wheel as a Rotation2d
     */
    public Rotation2d getSteeringAngle() {

        // CODE: Read encoder position from steering SparkMax and return Rotation2d.
        // The PositionConversionFactor is set so SparkMax returns radians, the default constructor of Rotation2d wants radians.

        return new Rotation2d(0);
    }

    /**
     * Gets a reading from the Lamprey and updates the SparkMax encoder (interal NEO encoder).
     * This is specific to Swerge. Other methods need to be written for other hardware.
     */
    public void updateSteeringFromLamprey() {
        
        // CODE: You can attempt this if you want but this will probably be done together in the 2nd or 3rd meeting.
        // CODE: Read value from Lamprey and set internal Neo encoder for the steering SparkMax (but need to add an offset first)

    }

    /**
     * Standard stop motors method for every subsystem.
     */
    public void stopMotors() {

        // CODE: Call the stopMotors method in the CANSparkMax (provided with all WPILib motor controller objects)

    }
}
