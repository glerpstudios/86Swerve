package frc.robot.resistanceswerve.subsystems;

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
 * This class supports both SparkMax and TalonFX motor controllers and their respective encoders.
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
     * The location of the module relative to the robot center of the robot's chassis.
     */
    Translation2d moduleLocation;

    /**
     * Creates a new SwerveModule.
     * @param encodeType The type of encoder being used ("SparkMax" or "TalonFX").
     * @param driveMotorID The ID of the drive motor.
     * @param drivePIDconstants The PID constants for the drive motor in the order [KP, KI, KD].
     * @param driveMotorType The type of the drive motor (MotorType.kBrushless or MotorType.kBrushed).
     * @param turnMotorID The ID of the turn motor.
     * @param turnPIDconstants The PID constants for the turn motor in the order [KP, KI, KD].
     * @param turnMotorType The type of the turn motor (MotorType.kBrushless or MotorType.kBrushed).
     * @param moduleLoc The location of the module relative to the robot center of the robot's chassis.
     */
    public SwerveModule(String encodeType, 
                        int driveMotorID, 
                        double[] drivePIDconstants, 
                        MotorType driveMotorType, 
                        int turnMotorID, 
                        double [] turnPIDconstants, 
                        MotorType turnMotorType,
                        Translation2d moduleLoc
                        ) 
    {
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

        // Set the module location
        this.moduleLocation = moduleLoc;
    }

    /**
     * Gets the location of the module relative to the robot center of the robot's chassis.
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
     * Gets the current state of the swerve module, which is the speed and angle of the module.
     * @return The speed and angle of the module as a SwerveModuleState object.
     */
    public SwerveModuleState getState() {
        // Speed in meters per second, angle in radians
        double speed = 0;
        double angle = 0;

        // Get speed and angle based on encoder type
        if (encoderType == "SparkMax") {
            speed = driveMotorSparkMax.getEncoder().getVelocity();
            angle = turnMotorSparkMax.getEncoder().getPosition() * 2 * Math.PI;
        } else if (encoderType == "TalonFX") {
            speed = driveMotorTalon.getVelocity().getValueAsDouble();
            angle = turnMotorTalon.getPosition().getValueAsDouble() * 2 * Math.PI;
        }

        // Combine speed and angle into a SwerveModuleState object
        return new SwerveModuleState(speed, new edu.wpi.first.math.geometry.Rotation2d(angle));
    }

    /**
     * Sets the desired state of the swerve module, which is the speed and angle of the module.
     * @param state The desired speed and angle of the module as a SwerveModuleState object.
     */
    public void setDesiredState(SwerveModuleState state) {
        // Using the drive PID set in the constructor, calculate the next speed of the drive motor
        double driveOutput = drivePID.calculate(getState().speedMetersPerSecond, state.speedMetersPerSecond);
        // Using the turn PID set in the constructor, calculate the next position of the turn motor
        double turnOutput = turnPID.calculate(getState().angle.getRadians(), state.angle.getRadians());

        if (encoderType == "SparkMax") {
            driveMotorSparkMax.set(driveOutput);
            turnMotorSparkMax.set(turnOutput);
        } else if (encoderType == "TalonFX") {
            driveMotorTalon.set(driveOutput);
            turnMotorTalon.set(turnOutput);
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

}
