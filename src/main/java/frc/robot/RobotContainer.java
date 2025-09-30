// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot._resistanceswerve.subsystems.*;
import frc.robot._resistanceswerve.commands.*;
import frc.robot.OperatorInput;
import frc.robot.subsystems.*;
import com.ctre.phoenix6.hardware.Pigeon2;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SwerveModule frontLeft = new SwerveModule("SparkMax",
      1,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      2,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      new Translation2d(),
      new Rotation2d());
  SwerveModule frontRight = new SwerveModule("SparkMax",
      3,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      4,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      new Translation2d(),
      new Rotation2d());
  SwerveModule backLeft = new SwerveModule("SparkMax",
      5,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      6,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      new Translation2d(),
      new Rotation2d());
  SwerveModule backRight = new SwerveModule("SparkMax",
      7,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      8,
      new double[] { 1, 0, 0 },
      MotorType.kBrushed,
      new Translation2d(),
      new Rotation2d());

  SwerveDrive drive = new SwerveDrive(frontLeft, frontRight, backLeft, backRight, new Pigeon2(9),
      10,
      11,
      true,
      1,
      new PIDController(1, 0, 0),
      new PIDController(1, 0, 0),
      new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(1, 1)));

  OperatorInput input = new OperatorInput();

  Vision vision = new Vision();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    drive.updateRobotPoseOutsideOdometry(
      () -> {
        return vision.getPose();
      }
    );
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // When button 1 is pressed, 
    input.js1.onTrue(new SwerveDriveToPoseAroundNoGoZonesCommand(
        drive, 
        6, 
        new Pose2d(),
        drive.getHolonomicDriveController(), 
        Constants.SwerveConstants.NO_GO_ZONES, 
        1, 
        1
        ));

    input.js2.onTrue(new SwerveDriveDefenseCommand(
      drive, 
      "active", 
      3
      ));
  }
}
