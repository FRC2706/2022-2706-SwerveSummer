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
public final class FConfig {
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
