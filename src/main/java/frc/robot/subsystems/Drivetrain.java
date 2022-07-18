package frc.robot.subsystems;

import java.util.Map;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

/**
 * Subsystem representing the swerve drivetrain
 */
public class Drivetrain extends SubsystemBase {

            
    // value controlled on shuffleboard to stop the jerkiness of the robot by limiting its accelera``tion
    public NetworkTableEntry maxAccel;
    public NetworkTableEntry speedLimitFactor;

    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double TRACKWIDTH_METERS = 0.6;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double WHEELBASE_METERS = 0.6;

    /**
     * The maximum voltage that will be delivered to the drive motors.
     * <p>
     * This can be reduced to cap the robot's maximum speed. Typically, this is
     * useful during initial testing of the robot.
     */
    public static final double MAX_VOLTAGE = 12.0;

    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4_L1.getDriveReduction() *
            SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0);

    /**
     * The model representing the drivetrain's kinematics
     */
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            // Front left
            new Translation2d(TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Front right
            new Translation2d(TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-TRACKWIDTH_METERS / 2.0, WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-TRACKWIDTH_METERS / 2.0, -WHEELBASE_METERS / 2.0));

    // These are our modules. We set them in the constructor.
    private SwerveModule m_frontLeftModule;
    private SwerveModule m_frontRightModule;
    private SwerveModule m_backLeftModule;
    private SwerveModule m_backRightModule;

    private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    private ShuffleboardTab tab;

    // Swerve module states - contains speed(m/s) and angle for each swerve module
    SwerveModuleState[] m_states;

    /**
     * Create a new swerve drivetrain
     * 
     * @param frontLeftModule  Front-left swerve module
     * @param frontRightModule Front-right swerve module
     * @param backLeftModule   Back-left swerve module
     * @param backRightModule  Back-right swerve module
     * @param navx             Pigeon IMU
     */
    public Drivetrain() {

        // SmartDashboard.putData("Field", m_field);

        tab = Shuffleboard.getTab("Drivetrain");

        resetModules(NeutralMode.Brake);

                /**Acceleration Limiting Slider*/
        maxAccel = tab.addPersistent("Max Acceleration", 0.05)
        .withPosition(8, 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 0.5))
        .getEntry();
        speedLimitFactor = tab.addPersistent("SpeedLimitFactor", 0.75)
        .withPosition(8, 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 0.75))
        .getEntry();
        tab.add("Reset Drivetrain", new InstantCommand(()->{resetModules(NeutralMode.Brake);}))
        .withPosition(0,0)
        .withSize(2, 1);
    }

    private void resetModules(NeutralMode nm) {

        final Mk4SwerveModuleHelper.GearRatio DRIVE_RATIO = Mk4SwerveModuleHelper.GearRatio.L1;

        TalonFX temp1 = new TalonFX(RobotMap.CANID.FL_DRIVE_FALCON);
        TalonFX temp2 = new TalonFX(RobotMap.CANID.FR_DRIVE_FALCON);
        TalonFX temp3 = new TalonFX(RobotMap.CANID.BL_DRIVE_FALCON);
        TalonFX temp4 = new TalonFX(RobotMap.CANID.BR_DRIVE_FALCON);

        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                DRIVE_RATIO, RobotMap.CANID.FL_DRIVE_FALCON, RobotMap.CANID.FL_STEER_FALCON,
                RobotMap.CANID.FL_STEER_ENCODER, -Math.toRadians(155 + 180));
        

        m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                DRIVE_RATIO, RobotMap.CANID.FR_DRIVE_FALCON, RobotMap.CANID.FR_STEER_FALCON,
                RobotMap.CANID.FR_STEER_ENCODER, -Math.toRadians(94 + 180));

        m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                DRIVE_RATIO, RobotMap.CANID.BL_DRIVE_FALCON, RobotMap.CANID.BL_STEER_FALCON,
                RobotMap.CANID.BL_STEER_ENCODER, -Math.toRadians(200 + 180));

        m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                DRIVE_RATIO, RobotMap.CANID.BR_DRIVE_FALCON, RobotMap.CANID.BR_STEER_FALCON,
                RobotMap.CANID.BR_STEER_ENCODER, -Math.toRadians(135 + 180));
        

        temp1.setNeutralMode(nm);
        temp2.setNeutralMode(nm);
        temp3.setNeutralMode(nm);
        temp4.setNeutralMode(nm);
    }

    /**
     * Control the drivetrain
     * 
     * @param translation   X/Y translation, in meters per second
     * @param rotation      Rotation, in radians per second
     * @param fieldOriented Boolean indicating if directions are field- or
     *                      robot-oriented
     */
    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {

        // correct axes of drive - determined from field testing
        // Feb 10 2022
        // flip sign of y axis speed
        // flip sign of rotation speed
        Translation2d newtranslation = new Translation2d(translation.getX(),
                -translation.getY());
        Double newrotation = -rotation;

        // not sure what this line was intended to do. KN Feb 11/2022
        // rotation *= 2.0 / Math.hypot(WHEELBASE_METERS, TRACKWIDTH_METERS);

        // determine chassis speeds
        if (fieldOriented) {
            m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(newtranslation.getX(),
                    newtranslation.getY(),
                    newrotation,
                    Rotation2d.fromDegrees(RobotContainer.gyro.getYaw()));
        } else {
            m_chassisSpeeds = new ChassisSpeeds(newtranslation.getX(),
                    newtranslation.getY(),
                    newrotation);
        }
    }

    @Override
    public void periodic() {

        m_states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(m_states, MAX_VELOCITY_METERS_PER_SECOND);
        SmartDashboard.putString("Speeds", m_chassisSpeeds.toString());

        m_frontLeftModule.set(m_states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_states[0].angle.getRadians());
        m_frontRightModule.set(m_states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_states[1].angle.getRadians());
        m_backLeftModule.set(m_states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_states[2].angle.getRadians());
        m_backRightModule.set(m_states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                m_states[3].angle.getRadians());
    }

    // -------------------- Kinematics and Swerve Module Status Public Access
    // Methods --------------------

    /** Returns kinematics of drive system */
    public SwerveDriveKinematics getKinematics() {
        return m_kinematics;
    }

    /**
     * Returns speed and angle status of all swerve modules
     * Returns array of length of of SwerveModuleStates
     */
    public SwerveModuleState[] getSwerveStates() {

        // create array of module states to return
        SwerveModuleState[] states = new SwerveModuleState[4];
        
        states[0] = new SwerveModuleState();
        states[0].speedMetersPerSecond = m_frontLeftModule.getDriveVelocity();
        states[0].angle = new Rotation2d(m_frontLeftModule.getSteerAngle());

        states[1] = new SwerveModuleState();
        states[1].speedMetersPerSecond = m_frontRightModule.getDriveVelocity();
        states[1].angle = new Rotation2d(m_frontRightModule.getSteerAngle());

        states[2] = new SwerveModuleState();
        states[2].speedMetersPerSecond = m_backLeftModule.getDriveVelocity();
        states[2].angle = new Rotation2d(m_backLeftModule.getSteerAngle());

        states[3] = new SwerveModuleState();
        states[3].speedMetersPerSecond = m_backRightModule.getDriveVelocity();
        states[3].angle = new Rotation2d(m_backRightModule.getSteerAngle());

        return states;
    }

} // end class Drivetrain