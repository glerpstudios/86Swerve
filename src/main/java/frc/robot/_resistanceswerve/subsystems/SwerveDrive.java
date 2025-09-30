/**
 * SwerveDrive.java
 *
 * Description: Subsystem handling a robot chassis of four swerve modules as a whole.
 *
 * Author(s): Samuel Sapatla
 * Additional Authors: (add names here as needed)
 *
 * Date Created: 2025-09-29
 * Last Modified: 2025-09-29
 */
package frc.robot._resistanceswerve.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot._resistanceswerve.util.ExternalOdometryProvider;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Timer;

/**
 * Represents the full swerve drive subsystem of the robot.
 * <p>
 * This class coordinates four {@link SwerveModule} instances (front left, front
 * right,
 * back left, back right) to provide holonomic motion control. It integrates
 * kinematics, odometry, and gyro data to track and command the robot's
 * position and orientation on the field.
 * <p>
 * Features:
 * <ul>
 * <li>
 * Converts chassis-level commands ({@link ChassisSpeeds}) into individual
 * wheel states via {@link SwerveDriveKinematics}.
 * </li>
 * <li>
 * Maintains a live estimate of robot pose using {@link SwerveDriveOdometry}
 * and an onboard gyro ({@link com.ctre.phoenix6.hardware.Pigeon2}).
 * </li>
 * <li>
 * Supports optional external odometry providers (e.g., vision systems) for
 * sensor fusion and more accurate pose tracking.
 * </li>
 * <li>
 * Provides trajectory-following support using a
 * {@link HolonomicDriveController} with tunable {@link PIDController}
 * and {@link ProfiledPIDController} loops for X, Y, and theta control.
 * </li>
 * <li>
 * Safety features such as maximum speed/rotation clamping and an “X lock”
 * formation to resist pushing when stopped.
 * </li>
 * </ul>
 * <p>
 * This class is designed to be the top-level interface for driving and
 * commanding a holonomic swerve drivetrain within <b>ResistanceSwerve</b>.
 * <p>
 * List of behaviors (called directly by commands or the robot container):
 * <ul>
 * <li>{@code drive(ChassisSpeeds, boolean)} - Drives the robot with given
 * chassis speeds (field- or robot-relative).</li>
 * <li>{@code stopModules()} - Stops all wheel motion while holding wheel
 * angles.</li>
 * <li>{@code getPoseByOdometry()} - Returns the robot's pose estimate from
 * odometry.</li>
 * <li>{@code changeOdometryPose(Pose2d)} - Resets odometry to a known field
 * position.</li>
 * <li>{@code updateRobotPoseOutsideOdometry(ExternalOdometryProvider)} -
 * Injects external pose estimates (e.g., vision).</li>
 * <li>{@code getHolonomicDriveController()} - Accesses the trajectory-following
 * controller.</li>
 * <li>{@code setXLockFormation()} - Locks wheels in an “X” pattern to resist
 * external forces.</li>
 * </ul>
 */
public class SwerveDrive extends SubsystemBase {
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
     * This is used to scale the chassis angular velocity to the maximum angular
     * velocity of the robot.
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
     * This is a functional interface that provides the current pose of the robot
     * from an external source.
     * This can be used to integrate vision-based odometry or other sensor fusion
     * methods.
     */
    ExternalOdometryProvider externalOdometryProvider;

    // === Safety configuration ===
    /**
     * The maximum allowed chassis speed in meters per second.
     */
    private static final double MAX_ALLOWED_CHASSIS_SPEED = 4.5; // m/s
    /**
     * The maximum allowed chassis angular speed in radians per second.
     */
    private static final double MAX_ALLOWED_ANGULAR_SPEED = Math.PI; // rad/s

    /**
     * Creates a new SwerveDrive subsystem.
     * This subsystem is responsible for controlling a swerve drive robot.
     * A holonomic drive controller is necessary for trajectory following.
     * 
     * @param frontLeft                             The front left swerve module.
     * @param frontRight                            The front right swerve module.
     * @param backLeft                              The back left swerve module.
     * @param backRight                             The back right swerve module.
     * @param pigeonGyro                            The gyro sensor for the swerve
     *                                              drive. This is used to get the
     *                                              robot's heading.
     * @param maxSpeedMetersPerSecond               The maximum speed of the robot
     *                                              in meters per second.
     * @param maxAngularVelocityRadiansPerSecond    The maximum angular velocity of
     *                                              the robot in radians per second.
     * @param useExternalOdometryUpdates            Whether to use external odometry
     *                                              updates.
     * @param intervalBetweenExternalUpdatesSeconds The time interval between
     *                                              external odometry updates in
     *                                              seconds.
     * @param robotXPID                             The PID controller for the
     *                                              robot's x control. Used for
     *                                              creating the holonomic drive
     *                                              controller.
     * @param robotYPID                             The PID controller for the
     *                                              robot's y control. Used for
     *                                              creating the holonomic drive
     *                                              controller.
     * @param robotThetaPID                         The profiled PID controller for
     *                                              the robot's theta (heading)
     *                                              control. Used for creating the
     *                                              holonomic drive controller.
     */
    public SwerveDrive(SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
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
        this.kinematics = new SwerveDriveKinematics(
            frontLeft.getModuleLocation(),
            frontRight.getModuleLocation(),
            backLeft.getModuleLocation(),
            backRight.getModuleLocation()
        );
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
        
        // If the wheel base and track width have been set, use those for kinematics!!


        // Set up the Holonomic DriveController and profiled PID Controllers
        this.robotXController = robotXPID;
        this.robotYController = robotYPID;
        this.robotThetaController = robotThetaPID;

        this.robotThetaController.reset(0);

        robotThetaController.enableContinuousInput(-Math.PI, Math.PI);
        this.holonomicController = new HolonomicDriveController(robotXController, robotYController,
                robotThetaController);
    }

    /**
     * Drives the robot with the given chassis speeds.
     * 
     * @param speeds        The desired chassis speeds.
     * @param fieldRelative Whether the speeds are field relative or robot relative.
     */
    public void drive(ChassisSpeeds speeds, boolean fieldRelative) {
        if (!validateChassisSpeeds(speeds)) {
            System.out.println("[SwerveDrive] Invalid chassis speeds, stopping modules.");
            stopModules();
            return;
        }

        ChassisSpeeds robotRelativeSpeeds = fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds.vxMetersPerSecond,
                speeds.vyMetersPerSecond,
                speeds.omegaRadiansPerSecond,
                pigeonGyro.getRotation2d())
                : speeds;

        // Clamp speeds to safety values
        double scaledVx = Math.max(-MAX_ALLOWED_CHASSIS_SPEED,
                Math.min(robotRelativeSpeeds.vxMetersPerSecond, MAX_ALLOWED_CHASSIS_SPEED));
        double scaledVy = Math.max(-MAX_ALLOWED_CHASSIS_SPEED,
                Math.min(robotRelativeSpeeds.vyMetersPerSecond, MAX_ALLOWED_CHASSIS_SPEED));
        double scaledOmega = Math.max(-MAX_ALLOWED_ANGULAR_SPEED,
                Math.min(robotRelativeSpeeds.omegaRadiansPerSecond, MAX_ALLOWED_ANGULAR_SPEED));

        var moduleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(scaledVx, scaledVy, scaledOmega));
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_ALLOWED_CHASSIS_SPEED);

        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    /**
     * Stops all swerve modules by setting their desired state to zero speed while
     * maintaining their current angle.
     */
    public void stopModules() {
        frontLeft.setDesiredState(new SwerveModuleState(0, frontLeft.getState().angle));
        frontRight.setDesiredState(new SwerveModuleState(0, frontRight.getState().angle));
        backLeft.setDesiredState(new SwerveModuleState(0, backLeft.getState().angle));
        backRight.setDesiredState(new SwerveModuleState(0, backRight.getState().angle));
    }

    /**
     * Validates the given chassis speeds to ensure they are within safe limits.
     * 
     * @param speeds The chassis speeds to validate.
     * @return True if the speeds are valid, false otherwise.
     */
    private boolean validateChassisSpeeds(ChassisSpeeds speeds) {
        if (Double.isNaN(speeds.vxMetersPerSecond) || Double.isNaN(speeds.vyMetersPerSecond))
            return false;
        if (Math.abs(speeds.vxMetersPerSecond) > MAX_ALLOWED_CHASSIS_SPEED)
            return false;
        if (Math.abs(speeds.vyMetersPerSecond) > MAX_ALLOWED_CHASSIS_SPEED)
            return false;
        if (Math.abs(speeds.omegaRadiansPerSecond) > MAX_ALLOWED_ANGULAR_SPEED)
            return false;
        return true;
    }

    /**
     * Updates the odometry of the robot.
     * This should be called periodically to update the robot's position on the
     * field.
     * 
     * @return The current pose of the robot.
     */
    public Pose2d getPoseByOdometry() {
        return odometry.getPoseMeters();
    }

    /**
     * Gets the current pose from the external odometry provider.
     * If no external provider is set, falls back to odometry-based pose.
     * 
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
     * This should be called periodically to update the robot's position on the
     * field.
     * 
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
     * This can be used to integrate vision-based odometry or other sensor fusion
     * methods.
     * It MUST return null if the external provider is not confident in its pose
     * estimation.
     * Example usage:
     * {@code swerveDrive.updateRobotPoseOutsideOdometry(() -> visionSystem.getEstimatedPose());}
     * 
     * @param provider The external odometry provider.
     */
    public void updateRobotPoseOutsideOdometry(ExternalOdometryProvider provider) {
        this.externalOdometryProvider = provider;
    }

    /**
     * Returns the holonomic drive controller for trajectory following.
     * 
     * @return The holonomic drive controller.
     */
    public HolonomicDriveController getHolonomicDriveController() {
        return holonomicController;
    }

    /**
     * Resets the odometry of the robot to the origin (0,0) with a heading of 0
     * degrees.
     * This should be called when the robot is placed on the field at the start of a
     * match.
     */
    void zeroHeading() {
        pigeonGyro.setYaw(0);
    }

    public void setXLockFormation() {
        // Angle wheels into an "X" shape (45° and -45°), zero speed
        SwerveModuleState fl = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
        SwerveModuleState fr = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        SwerveModuleState bl = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
        SwerveModuleState br = new SwerveModuleState(0, Rotation2d.fromDegrees(45));

        frontLeft.setDesiredState(fl);
        frontRight.setDesiredState(fr);
        backLeft.setDesiredState(bl);
        backRight.setDesiredState(br);
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
                if (newPose != null) {
                    changeOdometryPose(newPose);
                }
                // Reset the timer for the next update
                updateTimer.reset();
            }
        }
    }
}
