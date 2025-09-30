package frc.robot._resistanceswerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot._resistanceswerve.subsystems.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.Trajectory.State;

/**
 * A command that resists external pushing forces by locking or actively
 * holding the robot’s position.
 * <p>
 * Two defense modes are supported:
 * <ul>
 *   <li><b>Passive:</b> Locks the wheels in an “X” formation, making the
 *       robot physically harder to push without actively driving.</li>
 *   <li><b>Active:</b> Records the robot’s pose at initialization and
 *       continually drives back toward that pose using the
 *       {@link HolonomicDriveController} to reject displacement.</li>
 * </ul>
 * <p>
 * Optional duration can be provided, after which the command will end.
 * Otherwise, it will continue until interrupted.
 * <p>
 * Typical usage:
 * <ul>
 *   <li>Endgame defense when holding a position on the field.</li>
 *   <li>Resisting collisions during autonomous scoring.</li>
 *   <li>Fallback safety state when robot should remain stationary.</li>
 * </ul>
 */
public class SwerveDriveDefenseCommand extends Command {

    /**
     * The swerve drive subsystem
     */
    SwerveDrive swerveDrive;

    /**
     * Either "active" or "passive"
     * If active, the robot will actively resist being pushed, driving to the position where the command was originally called.
     * If passive, the robot will lock its wheels in an X formation to resist being pushed.
     */
    String defenseMode;

    /**
     * Duration in seconds to hold the defense position.
     * It is an optional parameter in the constructor. If it is not provided, the command will run until interrupted.
     */
    double durationSeconds = -1;

    /**
     * Keeps track of how long the command has been running.
     */
    Timer timer = new Timer();

    /**
     * The pose where the robot should hold position if in active defense mode.
     */
    private Pose2d holdPose;

    /**
     * Constructor for the SwerveDriveDefenseCommand.
     * This command will make the robot resist being pushed.
     * If the defenseMode is "active", the robot will actively resist being pushed, driving to the position where the command was originally called.
     * If the defenseMode is "passive", the robot will lock its wheels in an X formation to resist being pushed.
     * @param swerveDrive The swerve drive subsystem
     * @param defenseMode Either "active" or "passive"
     */
    public SwerveDriveDefenseCommand(SwerveDrive swerveDrive, String defenseMode) {
        // Initialize all the variables
        this.swerveDrive = swerveDrive;
        this.defenseMode = defenseMode;
        addRequirements(swerveDrive);
    }

    /**
     * Constructor for the SwerveDriveDefenseCommand.
     * This command will make the robot resist being pushed.
     * If the defenseMode is "active", the robot will actively resist being pushed, driving to the position where the command was originally called.
     * If the defenseMode is "passive", the robot will lock its wheels in an X formation to resist being pushed.
     * @param swerveDrive The swerve drive subsystem
     * @param defenseMode Either "active" or "passive"
     * @param durationSeconds Duration in seconds to hold the defense position.
     */
    public SwerveDriveDefenseCommand(SwerveDrive swerveDrive, String defenseMode, double durationSeconds) {
        // Initialize all the variables
        this.swerveDrive = swerveDrive;
        this.defenseMode = defenseMode;
        this.durationSeconds = durationSeconds;
        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (defenseMode.equals("active")) {
            // Snapshot current pose as the hold position
            holdPose = swerveDrive.getPoseByOdometry();
        } else if (defenseMode.equals("passive")) {
            // Set wheels into X-formation immediately
            swerveDrive.setXLockFormation();
        }
    }

    @Override
    public void execute() {
        if (defenseMode.equals("active")) {
            // Drive back toward the held pose using holonomic controller

            Pose2d currentPose = swerveDrive.getPoseByOdometry();
            // Create a state with zero velocity at the hold pose
            // Curvature in rad/m is 0 because we want to hold position, not follow a path
            State holdState = new State(0, 0, 0, holdPose, 0);

            ChassisSpeeds correction = swerveDrive.getHolonomicDriveController()
                    .calculate(currentPose, holdState, new Rotation2d(0)); // No rotation target

            swerveDrive.drive(correction, true);
        } else if (defenseMode.equals("passive")) {
            // Refresh X-lock (in case modules drift)
            swerveDrive.setXLockFormation();
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stopModules();
    }

    @Override
    public boolean isFinished() {
        if (durationSeconds > 0) {
            return timer.hasElapsed(durationSeconds);
        }
        return false; // run until interrupted otherwise
    }
}
