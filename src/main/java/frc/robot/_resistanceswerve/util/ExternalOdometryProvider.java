/**
 * ExternalOdometryProvider.java
 *
 * Description: Interface allowing for custom external pose estimation code.
 *
 * Author(s): Samuel Sapatla
 * Additional Authors: (add names here as needed)
 *
 * Date Created: 2025-09-29
 * Last Modified: 2025-09-29
 */
package frc.robot._resistanceswerve.util;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A functional interface for providing external odometry data for the ResistanceSwerve.
 * This can be used to integrate odometry from sources other than the robot's built-in sensors,
 * such as vision systems or external tracking systems.
 */
@FunctionalInterface
public interface ExternalOdometryProvider {
    Pose2d getExternalPose();
}
