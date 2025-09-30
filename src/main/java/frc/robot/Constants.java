// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot._resistanceswerve.util.*;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class SwerveConstants {
    public static final NoGoZones.NoGoZone NO_GO_ZONE_A = new NoGoZones().new NoGoZone(new Pose2d(), new Pose2d(), new Pose2d());
    public static final NoGoZones.NoGoZone NO_GO_ZONE_B = new NoGoZones().new NoGoZone(new Pose2d(), new Pose2d(), new Pose2d());
    public static final NoGoZones NO_GO_ZONES = new NoGoZones(NO_GO_ZONE_A, NO_GO_ZONE_B);
  }
}
