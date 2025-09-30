/**
 * SwerveModule.java
 *
 * Description: Subsystem representing one swerve module.
 *
 * Author(s): Samuel Sapatla
 * Additional Authors: (add names here as needed)
 *
 * Date Created: 2025-09-29
 * Last Modified: 2025-09-29
 */
package frc.robot._resistanceswerve.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents a single swerve module with a drive motor and a turn motor.
 * This class supports both SparkMax and TalonFX motor controllers and their
 * respective encoders.
 * <p>
 * It includes safety features such as voltage clamping and motor temperature
 * monitoring to prevent overheating.
 * <p>
 * It uses PID controllers for precise control of both drive and turn motors.
 * <p>
 * This class is designed to be used within <b>ResistanceSwerve</b>'s larger swerve drive system.
 * <p>
 * List of behaviors (SHOULD ONLY BE CALLED BY SwerveDrive):
 * <ul>
 * <li>
 * {@code clampOutput()} - Clamps the output of the PID controllers within this module to a certain safe limit.
 * </li>
 * <li>
 * {@code checkMotorHealth()} - Disables a module if it overheats.
 * </li>
 * <li>
 * {@code getModuleLocationAsSwerveModuleState()} - Returns the module's position as a SwerveModulePosition object.
 * </li>
 * <li>
 * {@code getModuleLocation()} - Returns the module's position as a Translation2d object.
 * </li>
 * <li>
 * {@code getState()} - Returns the current speed and angle of the swerve module as a SwerveModuleState object.
 * </li>
 * <li>
 * {@code setDesiredState} - Tells the module to assume a certain speed and angle, within safe limits.
 * </li>
 * </ul>
 */
public class SwerveModule extends SubsystemBase {

    /*
     * The encoder for the swerve module's drive motor if using SparkMax.
     */
    SparkMax driveMotorSparkMax;
    /*
     * The encoder for the swerve module's turn motor if using SparkMax.
     */
    SparkMax turnMotorSparkMax;

    /*
     * The encoder for the swerve module's drive motor if using TalonFX.
     */
    TalonFX driveMotorTalon;
    /*
     * The encoder for the swerve module's turn motor if using TalonFX.
     */
    TalonFX turnMotorTalon;

    /*
     * The type of encoder being used ("SparkMax" or "TalonFX").
     */
    String encoderType;

    // PID controllers for drive and turn motors
    PIDController drivePID;
    PIDController turnPID;

    /*
     * The location of the module relative to the robot center of the robot's
     * chassis.
     */
    Translation2d moduleLocation;

    /**
     * Often times, the angle that a motor considers as 0 is not the same as the
     * angle that we consider as 0. This variable allows us to set a reference
     * angle for the module's turn motor. This angle is relative to the robot's
     * forward direction.
     */
    Rotation2d zeroAngle = new Rotation2d(0); // Zero angle reference

    // === Safety configuration constants ===
    /**
     * The maximum voltage to apply to the drive motors to prevent overheating
     */
    private static final double MAX_DRIVE_VOLTAGE = 10.0; // Volts (reduce from 12 for safety margin)
    /**
     * The maximum voltage to apply to the turn motors to prevent overheating
     */
    private static final double MAX_TURN_VOLTAGE = 8.0; // Volts
    /**
     * The maximum speed for the drive motor (m/s)
     */
    private static final double MAX_DRIVE_SPEED = 4.5; // m/s (example max)

    // Not sure whether to use or not
    /**
     * The maximum speed for the turn motor (rad/s)
     */
    //private static final double MAX_TURN_SPEED_RAD_PER_SEC = Math.PI; // rad/s

    /**
     * The minimum command threshold to prevent small commands that could cause
     * jittering; this is used to prevent small controller inputs from causing the
     * motors to move.
     */
    private static final double MINIMUM_COMMAND_THRESHOLD = 0.01; // Deadband

    // Motor temperature threshold (deg C) - adjust to match your motor spec
    private static final double MAX_SAFE_TEMP_C = 70.0;

    /**
     * Creates a new SwerveModule.
     * 
     * @param encodeType        The type of encoder being used ("SparkMax" or
     *                          "TalonFX").
     * @param driveMotorID      The ID of the drive motor.
     * @param drivePIDconstants The PID constants for the drive motor in the order
     *                          [KP, KI, KD].
     * @param driveMotorType    The type of the drive motor (MotorType.kBrushless or
     *                          MotorType.kBrushed).
     * @param turnMotorID       The ID of the turn motor.
     * @param turnPIDconstants  The PID constants for the turn motor in the order
     *                          [KP, KI, KD].
     * @param turnMotorType     The type of the turn motor (MotorType.kBrushless or
     *                          MotorType.kBrushed).
     * @param moduleLoc         The location of the module relative to the robot
     *                          center of the robot's chassis.
     * @param zeroAngle         The reference zero angle for the module's turn
     *                          motor, relative to the robot's forward direction.
     *                          This is used to correct for any offset between the
     *                          motor's zero position and the desired zero position.
     */
    public SwerveModule(String encodeType,
            int driveMotorID,
            double[] drivePIDconstants,
            MotorType driveMotorType,
            int turnMotorID,
            double[] turnPIDconstants,
            MotorType turnMotorType,
            Translation2d moduleLoc,
            Rotation2d zeroAngle) {
        // Initialize encoder type
        encoderType = encodeType;

        // Initialize motors based on encoder type
        if (encoderType == "SparkMax") {
            driveMotorSparkMax = new SparkMax(driveMotorID, driveMotorType);
            turnMotorSparkMax = new SparkMax(turnMotorID, turnMotorType);
        } else if (encoderType == "TalonFX") {
            driveMotorTalon = new TalonFX(driveMotorID);
            turnMotorTalon = new TalonFX(turnMotorID);
        }

        // Initialize PID controllers for drive and turn motors
        this.drivePID = new PIDController(drivePIDconstants[0], drivePIDconstants[1], drivePIDconstants[2]);
        this.turnPID = new PIDController(turnPIDconstants[0], turnPIDconstants[1], turnPIDconstants[2]);

        // Set the module location and zero angle
        this.moduleLocation = moduleLoc;
        this.zeroAngle = zeroAngle;
    }

    // === Safety helpers ===
    /**
     * Clamps the output to the maximum voltage and applies a deadband to prevent
     * jittering.
     * 
     * @param output     The raw output from the PID controller
     * @param maxVoltage The maximum voltage to apply to the motor.
     * @return The clamped output scaled to the motor controller's expected input
     *         range.
     */
    private double clampOutput(double output, double maxVoltage) {
        return Math.max(-1.0, Math.min(1.0, output)) * (maxVoltage / 12.0); // scale to % output
    }

    /**
     * Checks the health of the motors by monitoring their temperatures.
     * If a motor exceeds the safe temperature threshold, it disables the motor
     * outputs.
     * This method should be called periodically to ensure motor safety.
     */
    private void checkMotorHealth() {
        if ("SparkMax".equals(encoderType)) {
            double driveTemp = driveMotorSparkMax.getMotorTemperature();
            double turnTemp = turnMotorSparkMax.getMotorTemperature();
            if (driveTemp > MAX_SAFE_TEMP_C || turnTemp > MAX_SAFE_TEMP_C) {
                System.out.println("[SwerveModule] WARNING: Motor overheating! Disabling outputs.");
                driveMotorSparkMax.stopMotor();
                turnMotorSparkMax.stopMotor();
            }
        }
        // TalonFX has getDeviceTemp
        if ("TalonFX".equals(encoderType)) {
            double driveTemp = driveMotorTalon.getDeviceTemp().getValueAsDouble();
            double turnTemp = turnMotorTalon.getDeviceTemp().getValueAsDouble();
            if (driveTemp > MAX_SAFE_TEMP_C || turnTemp > MAX_SAFE_TEMP_C) {
                System.out.println("[SwerveModule] WARNING: Motor overheating! Disabling outputs.");
                driveMotorTalon.set(0);
                turnMotorTalon.set(0);
            }
        }
    }

    /**
     * Gets the location of the module relative to the robot center of the robot's
     * chassis.
     * 
     * @return The location of the module as a SwerveModulePosition object.
     */
    public SwerveModulePosition getModuleLocationAsSwerveModulePosition() {
        // Get distance in meters from origin, which is the center of the robot
        double distanceInMeters = moduleLocation.getNorm();
        // Get angle of the module
        Rotation2d angle = moduleLocation.getAngle();

        // Combine distance and angle into a SwerveModulePosition object
        return new SwerveModulePosition(distanceInMeters, angle);
    }

    /**
     * Gets the location of the module relative to the robot center of the robot's
     * chassis.
     * 
     * @return The location of the module as a Translation2d object.
     */
    public Translation2d getModuleLocation() {
        return moduleLocation;
    }

    /**
     * Gets the current state of the swerve module, which is the speed and angle of
     * the module.
     * 
     * @return The speed and angle of the module as a SwerveModuleState object.
     */
    public SwerveModuleState getState() {
        // Speed in meters per second, angle in radians
        double speed = 0;
        Rotation2d rawAngle;

        // Get speed and angle based on encoder type
        if (encoderType == "SparkMax") {
            speed = driveMotorSparkMax.getEncoder().getVelocity();
        } else if (encoderType == "TalonFX") {
            speed = driveMotorTalon.getVelocity().getValueAsDouble();
        }
        if ("SparkMax".equals(encoderType)) {
            rawAngle = new Rotation2d(turnMotorSparkMax.getEncoder().getPosition() * 2 * Math.PI);
        } else if ("TalonFX".equals(encoderType)) {
            rawAngle = new Rotation2d(turnMotorTalon.getPosition().getValueAsDouble() * 2 * Math.PI);
        } else {
            rawAngle = new Rotation2d();
        }

        // Apply offset correction
        Rotation2d correctedAngle = rawAngle.minus(zeroAngle);

        // Combine speed and angle into a SwerveModuleState object
        return new SwerveModuleState(speed, correctedAngle);
    }

    /**
     * Sets the desired state of the swerve module, which is the speed and angle of
     * the module.
     * 
     * @param state The desired speed and angle of the module as a SwerveModuleState
     *              object.
     */
    public void setDesiredState(SwerveModuleState state) {
        // Optimize the state to minimize rotation
        state.optimize(getState().angle);

        // Clamp desired speeds to maximum
        double desiredDriveSpeed = Math.min(state.speedMetersPerSecond, MAX_DRIVE_SPEED);

        // Add offset back in when calculating turn target
        double desiredTurnRadians = state.angle.plus(zeroAngle).getRadians();

        // Using the drive PID set in the constructor, calculate the next speed of the
        // drive motor
        double driveOutput = drivePID.calculate(getState().speedMetersPerSecond, desiredDriveSpeed);
        // Using the turn PID set in the constructor, calculate the next position of the
        // turn motor
        double turnOutput = turnPID.calculate(getState().angle.getRadians(), desiredTurnRadians);

        // Deadband small outputs
        if (Math.abs(driveOutput) < MINIMUM_COMMAND_THRESHOLD)
            driveOutput = 0;
        if (Math.abs(turnOutput) < MINIMUM_COMMAND_THRESHOLD)
            turnOutput = 0;

        if ("SparkMax".equals(encoderType)) {
            driveMotorSparkMax.set(clampOutput(driveOutput, MAX_DRIVE_VOLTAGE));
            turnMotorSparkMax.set(clampOutput(turnOutput, MAX_TURN_VOLTAGE));
        } else if ("TalonFX".equals(encoderType)) {
            driveMotorTalon.set(clampOutput(driveOutput, MAX_DRIVE_VOLTAGE));
            turnMotorTalon.set(clampOutput(turnOutput, MAX_TURN_VOLTAGE));
        }
    }

    /**
     * Resets the encoders of the swerve module to zero.
     */
    public void resetEncoders() {
        if (encoderType == "SparkMax") {
            driveMotorSparkMax.getEncoder().setPosition(0);
            turnMotorSparkMax.getEncoder().setPosition(0);
        } else if (encoderType == "TalonFX") {
            driveMotorTalon.setPosition(0);
            turnMotorTalon.setPosition(0);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        checkMotorHealth();
    }

}
