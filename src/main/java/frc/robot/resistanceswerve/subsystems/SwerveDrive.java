package frc.robot.resistanceswerve.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.resistanceswerve.util.ExternalOdometryProvider;

public class SwerveDrive extends SubsystemBase{
    /** 
     * The front left swerve module.
    */
    SwerveModule frontLeft;
    /** 
     * The front right swerve module.
    */
    SwerveModule frontRight;
    /** 
     * The back left swerve module.
    */
    SwerveModule backLeft;
    /** 
     * The back right swerve module.
    */
    SwerveModule backRight;

    /** 
     * The kinematics object for the swerve drive.
     * This is used to convert between chassis speeds and individual module states.
    */
    SwerveDriveKinematics kinematics;
    /** 
     * The odometry object for the swerve drive.
     * This is used to track the robot's position on the field.
    */
    SwerveDriveOdometry odometry;

    /**
     * The gyro sensor for the swerve drive.
     * This is used to get the robot's heading.
     * The heading is the angle of the robot relative to the field.
     */
    Pigeon2 pigeonGyro;

    /** 
     * The maximum speed of the robot in meters per second.
     * This is used to scale the chassis speeds to the maximum speed of the robot.
    */
    double maxSpeedMetersPerSecond;
    /** 
     * The maximum angular velocity of the robot in radians per second.
     * This is used to scale the chassis angular velocity to the maximum angular velocity of the robot.
    */
    double maxAngularVelocityRadiansPerSecond;

    /**
     * The PID controllers for the robot's x and y control.
     */
    PIDController robotXController, robotYController;

    /**
     * The profiled PID controller for the robot's theta (heading) control.
     */
    ProfiledPIDController robotThetaController;

    /**
     * The holonomic drive controller for the swerve drive.
     * Used for trajectory following.
     */
    HolonomicDriveController holonomicController;

    /**
     * Timer for external odometry updates.
     */
    Timer updateTimer = new Timer();

    /**
     * Whether to use external odometry updates.
     */
    boolean useExternalOdometryUpdates;

    /**
     * The time interval between external odometry updates in seconds.
     */
    double intervalBetweenExternalUpdatesSeconds;

    /**
     * The external odometry provider.
     * This is a functional interface that provides the current pose of the robot from an external source.
     * This can be used to integrate vision-based odometry or other sensor fusion methods.
     */
    ExternalOdometryProvider externalOdometryProvider;

    /**
     * Creates a new SwerveDrive subsystem.
     * This subsystem is responsible for controlling a swerve drive robot.
     * A holonomic drive controller is necessary for trajectory following.
     * @param frontLeft The front left swerve module.
     * @param frontRight The front right swerve module.
     * @param backLeft The back left swerve module.
     * @param backRight The back right swerve module.
     * @param kinematics The kinematics object for the swerve drive. This is used to convert between chassis speeds and individual module states.
     * @param pigeonGyro The gyro sensor for the swerve drive. This is used to get the robot's heading.
     * @param maxSpeedMetersPerSecond The maximum speed of the robot in meters per second.
     * @param maxAngularVelocityRadiansPerSecond The maximum angular velocity of the robot in radians per second.
     * @param useExternalOdometryUpdates Whether to use external odometry updates.
     * @param intervalBetweenExternalUpdatesSeconds The time interval between external odometry updates in seconds.
     * @param robotXPID The PID controller for the robot's x control. Used for creating the holonomic drive controller.
     * @param robotYPID The PID controller for the robot's y control. Used for creating the holonomic drive controller.
     * @param robotThetaPID The profiled PID controller for the robot's theta (heading) control. Used for creating the holonomic drive controller.
     */
    public SwerveDrive( SwerveModule frontLeft, 
                        SwerveModule frontRight, 
                        SwerveModule backLeft, 
                        SwerveModule backRight,
                        SwerveDriveKinematics kinematics, 
                        Pigeon2 pigeonGyro,
                        double maxSpeedMetersPerSecond, 
                        double maxAngularVelocityRadiansPerSecond,
                        boolean useExternalOdometryUpdates,
                        double intervalBetweenExternalUpdatesSeconds,
                        PIDController robotXPID,
                        PIDController robotYPID,
                        ProfiledPIDController robotThetaPID) 
    {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.kinematics = kinematics;
        this.pigeonGyro = pigeonGyro;
        this.maxSpeedMetersPerSecond = maxSpeedMetersPerSecond;
        this.maxAngularVelocityRadiansPerSecond = maxAngularVelocityRadiansPerSecond;

        // Initialize odometry with the kinematics and the initial gyro angle
        this.odometry = new SwerveDriveOdometry(kinematics, 
                                                pigeonGyro.getRotation2d(),
                                                new SwerveModulePosition[] {
                                                    frontLeft.getModuleLocationAsSwerveModulePosition(),
                                                    frontRight.getModuleLocationAsSwerveModulePosition(),
                                                    backLeft.getModuleLocationAsSwerveModulePosition(),
                                                    backRight.getModuleLocationAsSwerveModulePosition()
                                                });
        
        //Set up the Holonomic DriveController and profiled PID Controllers
        this.robotXController = robotXPID;
        this.robotYController = robotYPID;
        this.robotThetaController = robotThetaPID;

        this.robotThetaController.reset(0);

        robotThetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.holonomicController = new HolonomicDriveController(robotXController, robotYController, robotThetaController);
    }

    /**
     * Drives the robot with the given chassis speeds.
     * @param speeds The desired chassis speeds.
     * @param fieldRelative Whether the speeds are field relative or robot relative.
     */
    public void drive (ChassisSpeeds speeds, boolean fieldRelative) {
        // If field relative, convert the speeds to robot relative.
        // This involves rotating the speeds by the negative of the robot's heading.
        // Otherwise, use the speeds as is.
        ChassisSpeeds robotRelativeSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds.vxMetersPerSecond, 
            speeds.vyMetersPerSecond, 
            speeds.omegaRadiansPerSecond, 
            pigeonGyro.getRotation2d()) : speeds;

        // Scale the speeds to the maximum speed of the robot.
        // This is done to ensure that the robot does not exceed its maximum speed.
        double scaledVx = robotRelativeSpeeds.vxMetersPerSecond * maxSpeedMetersPerSecond;
        double scaledVy = robotRelativeSpeeds.vyMetersPerSecond * maxSpeedMetersPerSecond;
        double scaledOmega = robotRelativeSpeeds.omegaRadiansPerSecond * maxAngularVelocityRadiansPerSecond;

        // Get the module states from the chassis speeds
        var moduleStates = kinematics.toSwerveModuleStates(
            new ChassisSpeeds(scaledVx, scaledVy, scaledOmega)
        );

        // Set the module states to the individual modules
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    /**
     * Updates the odometry of the robot.
     * This should be called periodically to update the robot's position on the field.
     * @return The current pose of the robot.
     */
    public Pose2d getPoseByOdometry() {
        return odometry.getPoseMeters();
    }

    /**
     * Gets the current pose from the external odometry provider.
     * If no external provider is set, falls back to odometry-based pose.
     * @return The current pose of the robot.
     */
    private Pose2d getExternalOdometryPose() {
        if (externalOdometryProvider != null) {
            return externalOdometryProvider.getExternalPose();
        }
        return getPoseByOdometry(); // fallback
    }
    

    /**
     * Updates the odometry of the robot.
     * This should be called periodically to update the robot's position on the field.
     * @param newPose The new pose of the robot.
     */
    public void changeOdometryPose(Pose2d newPose) {
        odometry.resetPosition(pigeonGyro.getRotation2d(), 
                               new SwerveModulePosition[] {
                                   frontLeft.getModuleLocationAsSwerveModulePosition(),
                                   frontRight.getModuleLocationAsSwerveModulePosition(),
                                   backLeft.getModuleLocationAsSwerveModulePosition(),
                                   backRight.getModuleLocationAsSwerveModulePosition()
                               }, 
                               newPose);
    }

    /**
     * Gets the current pose from the external odometry provider.
     * This can be used to integrate vision-based odometry or other sensor fusion methods.
     * @param provider The external odometry provider.
     */
    public void updateRobotPoseOutsideOdometry(ExternalOdometryProvider provider) {
        this.externalOdometryProvider = provider;
    }

    /**
     * Returns the holonomic drive controller for trajectory following.
     * @return The holonomic drive controller.
     */
    public HolonomicDriveController getHolonomicDriveController() {
        return holonomicController;
    }

    /**
     * Resets the odometry of the robot to the origin (0,0) with a heading of 0 degrees.
     * This should be called when the robot is placed on the field at the start of a match.
     */
    void zeroHeading() {
        pigeonGyro.setYaw(0);
    }

    @Override
    public void periodic() {
        // Update the odometry with the current gyro angle and module positions
        odometry.update(pigeonGyro.getRotation2d(), 
                        new SwerveModulePosition[] {
                            frontLeft.getModuleLocationAsSwerveModulePosition(),
                            frontRight.getModuleLocationAsSwerveModulePosition(),
                            backLeft.getModuleLocationAsSwerveModulePosition(),
                            backRight.getModuleLocationAsSwerveModulePosition()
                        });

        // Handle external odometry updates if enabled
        if (useExternalOdometryUpdates) {
            if (updateTimer.hasElapsed(intervalBetweenExternalUpdatesSeconds)) {
                // Here, you would get the new pose from your external odometry source
                // For example, from a vision system or other sensor fusion method
                Pose2d newPose = getExternalOdometryPose();

                // Update the odometry with the new pose
                changeOdometryPose(newPose);

                // Reset the timer for the next update
                updateTimer.reset();
            }
        }
    }
}
