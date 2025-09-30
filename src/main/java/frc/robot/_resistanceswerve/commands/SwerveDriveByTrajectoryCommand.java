/**
 * SwerveDriveByTrajectoryCommand.java
 *
 * Description: Command to drive the swerve drivetrain along a trajectory.
 *
 * Author(s): Samuel Sapatla
 * Additional Authors: (add names here as needed)
 *
 * Date Created: 2025-09-29
 * Last Modified: 2025-09-30
 */
package frc.robot._resistanceswerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot._resistanceswerve.subsystems.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command that drives the robot along a predefined {@link Trajectory}
 * using a {@link HolonomicDriveController}.
 * <p>
 * This command samples the trajectory over time and calculates the
 * chassis-level velocity targets (X, Y, and angular) that move the robot
 * along the path while maintaining desired orientation. These targets are
 * then sent to the {@link SwerveDrive} subsystem, which translates them
 * into individual swerve module states.
 * <p>
 * Typical usage:
 * <ul>
 *   <li>Autonomous routines that require following a generated path.</li>
 *   <li>Integration with PathPlanner or WPILib TrajectoryGenerator.</li>
 *   <li>Combining with event markers or command groups for complex autos.</li>
 * </ul>
 * <p>
 * This command ends automatically when the trajectory duration is complete.
 */
public class SwerveDriveByTrajectoryCommand extends Command {
    /**
     * The swerve drive subsystem.
     */
    SwerveDrive swerveDrive;

    /**
     * The trajectory to follow.
     */
    Trajectory trajectory;

    /**
     * The holonomic drive controller.
     * This is used to calculate the chassis speeds needed to follow the trajectory.
     * This is NOT the PID controllers for the swerve modules.
     */
    HolonomicDriveController holonomicDriveController;

    /**
     * Keeps track of the progress through the trajectory.
     */
    Timer timer = new Timer();

    /**
     * Creates a new SwerveDriveByTrajectoryCommand.
     * This command will follow the given trajectory using the given holonomic drive controller.
     * @param swerveDrive The swerve drive subsystem.
     * @param trajectory The trajectory to follow.
     * @param holonomicDriveController The holonomic drive controller to use.
     * @param fieldRelative Whether the robot should drive in field-relative mode.
     */
    public SwerveDriveByTrajectoryCommand(  SwerveDrive swerveDrive, 
                                            Trajectory trajectory, 
                                            HolonomicDriveController holonomicDriveController,
                                            boolean fieldRelative) {
        this.swerveDrive = swerveDrive;
        this.trajectory = trajectory;
        this.holonomicDriveController = holonomicDriveController;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        // Start the timer.
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Get the desired state from the trajectory at the current time.
        Trajectory.State desiredState = trajectory.sample(timer.get());

        // Get the current pose of the robot from the swerve drive subsystem.
        Pose2d currentPose = swerveDrive.getPoseByOdometry();

        // Calculate the desired chassis speeds using the holonomic drive controller.
        ChassisSpeeds chassisSpeeds = holonomicDriveController.calculate(currentPose, desiredState, desiredState.poseMeters.getRotation());

        // Pass the calculated chassis speeds to the swerve drive subsystem.
        // Field relative is false because the controller already accounts for it.
        swerveDrive.drive(chassisSpeeds, false);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the trajectory is complete.
        Pose2d error = trajectory.getStates()
                .get(trajectory.getStates().size() - 1) // final pose
                        .poseMeters
                .relativeTo(swerveDrive.getPoseByOdometry());

        return Math.abs(error.getX()) < 0.05 &&
                Math.abs(error.getY()) < 0.05 &&
                Math.abs(error.getRotation().getRadians()) < 0.05;
    }
}
