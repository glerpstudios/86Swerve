package frc.robot.resistanceswerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.resistanceswerve.subsystems.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;

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
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }
}
