/**
 * SwerveDriveByVectorCommand.java
 *
 * Description: Command to drive the swerve drivetrain by a translation vector.
 *
 * Author(s): Samuel Sapatla
 * Additional Authors: (add names here as needed)
 *
 * Date Created: 2025-09-29
 * Last Modified: 2025-09-29
 */
package frc.robot._resistanceswerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot._resistanceswerve.subsystems.*;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A command that moves the robot by a given vector delta using PID feedback
 * on X, Y, and rotation.
 * <p>
 * The target is expressed as a {@link Transform2d}, which defines a relative
 * translation and rotation offset from the robot's current pose. The command
 * calculates the error between the current pose and target pose, then uses
 * high-level {@link PIDController} instances (not module-level controllers)
 * to drive the robot toward the target.
 * <p>
 * Typical usage:
 * <ul>
 *   <li>Autonomous moves by a fixed offset (e.g., “strafe right 1m, rotate 90°”).</li>
 *   <li>Building blocks for multi-step autonomous sequences without needing
 *       a full trajectory generator.</li>
 *   <li>Quick corrections based on sensor input (e.g., vision alignment).</li>
 * </ul>
 * <p>
 * The command finishes once the robot is within position and rotation
 * tolerances of the target.
 */
public class SwerveDriveByVectorCommand extends Command{
    /**
     * The swerve drive subsystem.
     */
    SwerveDrive swerveDrive;

    /**
     * The target delta transformation for the robot to move.
     * This is the vector that the robot will move in.
     */
    Transform2d targetDelta;

    /**
     * The PID controllers for the x, y, and theta axes.
     * These are used to control the robot's movement to the target delta.
     * These are NOT the PID controllers for the swerve modules.
     * Rather, these are high-level PID controllers for the robot's overall movement.
     */
    PIDController xPID, yPID, thetaPID;

    /**
     * Whether the robot should drive in field-relative mode.
     */
    boolean fieldRelative;

    /**
     * Creates a new SwerveDriveByVectorCommand.
     * This command will move the robot in the direction of the target delta.
     * This command should be used for autonomous movement.
     * @param swerveDrive The swerve drive subsystem.
     * @param targetDelta The target delta translation for the robot to move.
     * @param xPID The PID controller for the x axis.
     * @param yPID The PID controller for the y axis.
     * @param thetaPID The PID controller for the theta axis.
     * @param fieldRelative Whether the robot should drive in field-relative mode.
     */
    public SwerveDriveByVectorCommand(  SwerveDrive swerveDrive, 
                                        Transform2d targetDelta, 
                                        PIDController xPID, 
                                        PIDController yPID, 
                                        PIDController thetaPID,
                                        boolean fieldRelative)
    {
        this.swerveDrive = swerveDrive;
        this.targetDelta = targetDelta;
        this.xPID = xPID;
        this.yPID = yPID;
        this.thetaPID = thetaPID;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // Get the current pose of the robot
        var currentPose = swerveDrive.getPoseByOdometry();

        // Calculate the target pose by adding the target delta to the current pose
        var targetPose = currentPose.plus(targetDelta);

        // Calculate the errors in x, y, and theta
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double thetaError = targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians();

        // Calculate the desired speeds using the PID controllers
        double xSpeed = xPID.calculate(0, xError);
        double ySpeed = yPID.calculate(0, yError);
        double rotSpeed = thetaPID.calculate(0, thetaError);

        // Create a new ChassisSpeeds object and pass it to the swerve drive subsystem
        swerveDrive.drive(
            fieldRelative ? 
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, currentPose.getRotation()) :
                new ChassisSpeeds(xSpeed, ySpeed, rotSpeed),
            fieldRelative
        );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0), false);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the robot is within a certain threshold of the target pose
        var currentPose = swerveDrive.getPoseByOdometry();
        var targetPose = currentPose.plus(targetDelta);

        double xError = Math.abs(targetPose.getX() - currentPose.getX());
        double yError = Math.abs(targetPose.getY() - currentPose.getY());
        double thetaError = Math.abs(targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians());

        // Thresholds can be adjusted as needed
        return (xError < 0.05) && (yError < 0.05) && (thetaError < 0.1);
    }
}
