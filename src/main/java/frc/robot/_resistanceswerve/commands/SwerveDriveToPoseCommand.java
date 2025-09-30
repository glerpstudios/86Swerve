/**
 * SwerveDriveToPose.java
 *
 * Description: Command to drive the swerve drivetrain to a certain field-relative pose.
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * A command that drives the robot directly to a specified {@link Pose2d} target
 * using independent PID controllers for X, Y, and rotation.
 * <p>
 * Unlike trajectory-based commands, this command does not generate or follow a
 * continuous path. Instead, it directly calculates the chassis speeds needed to
 * reduce the error between the current pose and target pose, with feedback
 * provided by the given {@link PIDController} instances.
 * <p>
 * Typical usage:
 * <ul>
 *   <li>Simple autonomous maneuvers where precise final positioning matters
 *       more than the intermediate path.</li>
 *   <li>Vision-based alignment (e.g., to AprilTags or targets) by setting a
 *       desired field-relative pose.</li>
 *   <li>Fine adjustments during autonomous routines without needing a full
 *       trajectory generator.</li>
 * </ul>
 * <p>
 * Features:
 * <ul>
 *   <li>High-level PID control — not per-module control — that works in
 *       field coordinates.</li>
 *   <li>Finishes automatically when the robot is within the configured
 *       tolerances of the target pose.</li>
 *   <li>Runs continuously until tolerance is met or the command is
 *       interrupted.</li>
 * </ul>
 * <p>
 * This command requires field-relative operation, as the target pose is
 * defined in field coordinates.
 */
public class SwerveDriveToPoseCommand extends Command{
    /**
     * The swerve drive subsystem.
     */
    SwerveDrive swerveDrive;

    /**
     * The target pose for the robot to move to.
     * This is the position and orientation that the robot will move to.
     */
    Pose2d targetPose;

    /**
     * The PID controllers for the x, y, and theta axes.
     * These are used to control the robot's movement to the target pose.
     * These are NOT the PID controllers for the swerve modules.
     * Rather, these are high-level PID controllers for the robot's overall movement.
     */
    PIDController xPID, yPID, thetaPID;

    /**
     * Creates a new SwerveDriveToPoseCommand.
     * This command will move the robot to the target pose.
     * This command should be used for autonomous movement.
     * This command can ONLY be used for field-relative driving.
     * This is because the target pose is defined in field coordinates.
     * @param swerveDrive The swerve drive subsystem.
     * @param targetPose The target pose for the robot to move to.
     * @param xPID The PID controller for the x axis.
     * @param yPID The PID controller for the y axis.
     * @param thetaPID The PID controller for the theta axis.
     */
    public SwerveDriveToPoseCommand(  SwerveDrive swerveDrive, 
                                      Pose2d targetPose, 
                                      PIDController xPID, 
                                      PIDController yPID, 
                                      PIDController thetaPID,
                                      boolean fieldRelative)
    {
        this.swerveDrive = swerveDrive;
        this.targetPose = targetPose;
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
        // Get the current pose of the robot.
        Pose2d currentPose = swerveDrive.getPoseByOdometry();

        // Calculate the chassis speeds needed to move to the target pose.
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
            xPID.calculate(currentPose.getX(), targetPose.getX()), 
            yPID.calculate(currentPose.getY(), targetPose.getY()), 
            thetaPID.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians())
        );

        // Pass the chassis speeds to the swerve drive subsystem.
        swerveDrive.drive(chassisSpeeds, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0), true);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the robot is within a certain tolerance of the target pose.
        return xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint();
    }
}
