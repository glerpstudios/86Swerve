package frc.robot.resistanceswerve.util;

import edu.wpi.first.math.geometry.Pose2d;

@FunctionalInterface
public interface ExternalOdometryProvider {
    Pose2d getExternalPose();
}
