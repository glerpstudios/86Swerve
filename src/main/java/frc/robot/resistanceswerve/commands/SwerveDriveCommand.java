package frc.robot.resistanceswerve.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.resistanceswerve.subsystems.*;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveDriveCommand extends Command {
    /**
     * The swerve drive subsystem.
     */
    SwerveDrive swerveDrive;

    // A supplier is used because the joystick values are constantly changing.
    /**
     * The supplier for the x speed of the robot in meters per second.
     */
    Supplier<Double> xSpeed;
    /**
     * The supplier for the y speed of the robot in meters per second.
     */
    Supplier<Double> ySpeed;
    /**
     * The supplier for the rotational speed of the robot in radians per second.
     */
    Supplier<Double> rotSpeed;
    
    /**
     * The supplier for whether the robot is driving in field-relative mode.
     * If true, the robot will drive in field-relative mode.
     * If false, the robot will drive in robot-relative mode.
     */
    Supplier<Boolean> fieldRelative;

    /*
     * Creates a new SwerveDriveCommand.
     * Can be used for both field-relative and robot-relative driving.
     */
    public SwerveDriveCommand(SwerveDrive swerveDrive, Supplier<Double> xSpeed, Supplier<Double> ySpeed, Supplier<Double> rotSpeed, Supplier<Boolean> fieldRelative) {
        this.swerveDrive = swerveDrive;
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotSpeed = rotSpeed;
        this.fieldRelative = fieldRelative;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        // Nothing to initialize
    }

    @Override
    public void execute() {
        // Use the supplier to get the current joystick values.
        // Then, create a new ChassisSpeeds object and pass it to the swerve drive subsystem.
        ChassisSpeeds newChassisSpeeds = fieldRelative.get() ? ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed.get(), 
            ySpeed.get(), 
            rotSpeed.get(), 
            swerveDrive.getPoseByOdometry().getRotation()
        ) : new ChassisSpeeds(
            xSpeed.get(), 
            ySpeed.get(), 
            rotSpeed.get()
        );
        // Raw speeds are used here because they are accounted for in SwerveDrive.drive();
        swerveDrive.drive(newChassisSpeeds, fieldRelative.get());
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0), false);
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own. It is interrupted by other commands.
        return false;
    }
}
