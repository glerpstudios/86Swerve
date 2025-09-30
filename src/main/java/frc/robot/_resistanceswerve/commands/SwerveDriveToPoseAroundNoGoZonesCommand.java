/**
 * SwerveDriveToPoseAroundNoGoZonesCommand.java
 *
 * Description: Command to drive the robot to a pose, avoiding certain NoGoZones.
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
import frc.robot._resistanceswerve.util.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.HolonomicDriveController;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.Timer;

/**
 * A command that drives the robot to a specified {@link Pose2d} target while
 * dynamically avoiding {@link NoGoZones} on the field.
 * <p>
 * This command constructs a trajectory from the robot’s current pose to the
 * desired pose. If the straight-line path intersects one or more no-go zones,
 * a detour path is calculated by skirting the polygon edges of the obstructing
 * zones, offset by a configurable buffer distance. The buffer ensures the
 * robot does not clip corners and accounts for its own dimensions.
 * <p>
 * Once a trajectory is generated, it is executed using a
 * {@link HolonomicDriveController} to produce chassis-level velocities,
 * which are sent to the {@link SwerveDrive} subsystem. The robot follows this
 * trajectory until it reaches the final pose or the command is interrupted.
 * <p>
 * Typical usage:
 * <ul>
 *   <li>Autonomous routines where the robot must navigate around field
 *       elements or dynamically defined restricted areas.</li>
 *   <li>Scenarios where straight-line driving to a target is not feasible
 *       due to game pieces, field geometry, or defensive robots.</li>
 *   <li>General obstacle avoidance when combined with updated no-go zone
 *       definitions at runtime.</li>
 * </ul>
 * <p>
 * Limitations:
 * <ul>
 *   <li>This command requires field-relative coordinates; it is not compatible
 *       with robot-relative driving.</li>
 *   <li>Pathfinding is simplified to polygon-edge detours and does not use a
 *       full graph-search algorithm.</li>
 *   <li>Works best for convex or mildly concave no-go zones.</li>
 * </ul>
 * <p>
 * The command ends automatically once the trajectory duration elapses.
 */
public class SwerveDriveToPoseAroundNoGoZonesCommand extends Command{
    /**
     * The swerve drive subsystem.
     */
    SwerveDrive swerveDrive;

    /**
     * The minimum distance to be held between the center of the robot and the no go zones.
     * This is to ensure that the robot does not collide with the no go zones.
     * This also makes sure that, for no go zones with sharp corners, the robot can make a rounded path around the corner.
     * This distance is in meters.
     */
    double bufferDistance;

    /**
     * The target pose for the robot to move to.
     * This is the position and orientation that the robot will move to.
     */
    Pose2d targetPose;

    /**
     * The holonomic drive controller.
     * This is used to calculate the chassis speeds needed to follow the trajectory.
     * This is NOT the PID controllers for the swerve modules.
     */
    HolonomicDriveController holonomicDriveController;

    /**
     * The no go zones that the robot must avoid.
     * These are areas on the field that the robot cannot enter.
     */
    NoGoZones noGoZones;

    /**
     * The trajectory between the robot's current pose and the target pose.
     * It will be calculated using a pathfinding algorithm that avoids the no go zones by the buffer distance.
     */
    Trajectory trajectory;

    /**
     * The maximum velocity of the robot in meters per second.
     * This is used to limit the speed of the robot when moving to the target pose.
     */
    double maxVelocityMetersPerSecond;

    /**
     * The maximum acceleration of the robot in meters per second squared.
     * This is used to limit the acceleration of the robot when moving to the target pose.
     */
    double maxAccelerationMetersPerSecondSquared;

    /**
     * A timer to track the progress of the trajectory.
     */
    Timer timer = new Timer();

    /**
     * Creates a new SwerveDriveToPoseAroundNoGoZones.
     * This command will move the robot to the target pose while skirting the edges of the no go zones in its path by the buffer distance.
     * This command should be used for autonomous movement.
     * This command can ONLY be used for field-relative driving.
     * This is because the target pose is defined in field coordinates.
     * @param swerveDrive The swerve drive subsystem.
     * @param bufferDistance The minimum distance to be held between the center of the robot and the no go zones, in meters.
     * @param targetPose The target pose for the robot to move to.
     * @param holonomicDriveController The holonomic drive controller to use.
     * @param zones The no go zones that the robot must avoid.
     * @param maxVelocityMetersPerSecond The maximum velocity of the robot in meters per second.
     * @param maxAccelerationMetersPerSecondSquared The maximum acceleration of the robot in meters per second squared.
     */
    public SwerveDriveToPoseAroundNoGoZonesCommand(  SwerveDrive swerveDrive, 
                                              double bufferDistance,
                                              Pose2d targetPose, 
                                              HolonomicDriveController holonomicDriveController,
                                              NoGoZones zones,
                                              double maxVelocityMetersPerSecond,
                                              double maxAccelerationMetersPerSecondSquared
                                              ) 
    {
        this.swerveDrive = swerveDrive;
        this.bufferDistance = bufferDistance;
        this.targetPose = targetPose;
        this.holonomicDriveController = holonomicDriveController;
        this.maxVelocityMetersPerSecond = maxVelocityMetersPerSecond;
        this.maxAccelerationMetersPerSecondSquared = maxAccelerationMetersPerSecondSquared;
        this.noGoZones = zones;
        addRequirements(swerveDrive);
    }

    /**
     * Finds the intersection points between a line segment and a polygon.
     * @param start The start point of the line segment.
     * @param end The end point of the line segment.
     * @param polygon The vertices of the polygon.
     * @return A list of intersection points.
     */
    private ArrayList<Pose2d> getLinePolygonIntersections(Pose2d start, Pose2d end, Pose2d[] polygon) {
        ArrayList<Pose2d> intersections = new ArrayList<>();
        
        double x1 = start.getX();
        double y1 = start.getY();
        double x2 = end.getX();
        double y2 = end.getY();
    
        int n = polygon.length;
        for (int i = 0; i < n; i++) {
            Pose2d v1 = polygon[i];
            Pose2d v2 = polygon[(i + 1) % n];
    
            double x3 = v1.getX();
            double y3 = v1.getY();
            double x4 = v2.getX();
            double y4 = v2.getY();
    
            // Line segments intersection formula
            double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (denom == 0) continue; // parallel
    
            double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
            double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;
    
            if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
                double ix = x1 + t * (x2 - x1);
                double iy = y1 + t * (y2 - y1);
                intersections.add(new Pose2d(ix, iy, start.getRotation()));
            }
        }
    
        return intersections;
    }
    
    /**
     * Walks along the edges of a polygon from an entry point to an exit point.
     * The entry and exit points should be on the polygon edges.
     * @param entry The entry point on the polygon edge.
     * @param exit The exit point on the polygon edge. 
     * @param vertices The vertices of the polygon.
     * @param clockwise Whether to walk clockwise or counterclockwise.
     * @return A list of waypoints along the polygon edges from entry to exit.
     */
    private ArrayList<Translation2d> walkPolygon(Pose2d entry, Pose2d exit, Pose2d[] vertices, boolean clockwise) {
        ArrayList<Translation2d> path = new ArrayList<>();
        int n = vertices.length;
    
        // Find closest edges for entry and exit
        int entryEdge = 0;
        double entryMinDist = Double.MAX_VALUE;
        int exitEdge = 0;
        double exitMinDist = Double.MAX_VALUE;
    
        for (int i = 0; i < n; i++) {
            Pose2d v1 = vertices[i];
            Pose2d v2 = vertices[(i + 1) % n];
    
            double entryDist = distancePointToSegment(entry, v1, v2);
            if (entryDist < entryMinDist) {
                entryMinDist = entryDist;
                entryEdge = i;
            }
    
            double exitDist = distancePointToSegment(exit, v1, v2);
            if (exitDist < exitMinDist) {
                exitMinDist = exitDist;
                exitEdge = i;
            }
        }
    
        // Build ordered list of vertices including entry/exit
        ArrayList<Pose2d> orderedVertices = new ArrayList<>();
        // Add entry point
        orderedVertices.add(entry);
    
        int idx = entryEdge;
        while (idx != exitEdge) {
            idx = clockwise ? (idx + 1) % n : (idx - 1 + n) % n;
            orderedVertices.add(vertices[idx]);
        }
    
        // Add exit point
        orderedVertices.add(exit);
    
        // Convert to Translation2d
        for (Pose2d p : orderedVertices) {
            path.add(new Translation2d(p.getX(), p.getY()));
        }
    
        return path;
    }
    
    /**
     * Distance from point p to line segment v-w
     */
    private double distancePointToSegment(Pose2d p, Pose2d v, Pose2d w) {
        double l2 = Math.pow(w.getX() - v.getX(), 2) + Math.pow(w.getY() - v.getY(), 2);
        if (l2 == 0) return Math.hypot(p.getX() - v.getX(), p.getY() - v.getY());
    
        double t = ((p.getX() - v.getX()) * (w.getX() - v.getX()) + (p.getY() - v.getY()) * (w.getY() - v.getY())) / l2;
        t = Math.max(0, Math.min(1, t));
        double projX = v.getX() + t * (w.getX() - v.getX());
        double projY = v.getY() + t * (w.getY() - v.getY());
        return Math.hypot(p.getX() - projX, p.getY() - projY);
    }
    
    
    
    private double pathLength(ArrayList<Translation2d> waypoints) {
        double len = 0;
        for (int i = 1; i < waypoints.size(); i++) {
            len += waypoints.get(i).getDistance(waypoints.get(i - 1));
        }
        return len;
    }

    /**
     * Returns the perpendicular distance from point p to the line defined by start -> end.
     */
    private double perpendicularDistance(Pose2d start, Pose2d end, Translation2d p) {
        double x0 = p.getX(), y0 = p.getY();
        double x1 = start.getX(), y1 = start.getY();
        double x2 = end.getX(), y2 = end.getY();
        double num = Math.abs((y2 - y1) * x0 - (x2 - x1) * y0 + x2*y1 - y2*x1);
        double den = Math.hypot(y2 - y1, x2 - x1);
        return num / den;
    }
    

    @Override
    public void initialize() {
        Pose2d currentPose = swerveDrive.getPoseByOdometry();
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<>();
    
        // Step 1: find intersecting zones
        ArrayList<Integer> intersectingZones = new ArrayList<>();
        for (int i = 0; i <= 100; i++) {
            double t = i / 100.0;
            double x = currentPose.getX() + t * (targetPose.getX() - currentPose.getX());
            double y = currentPose.getY() + t * (targetPose.getY() - currentPose.getY());
            Pose2d probe = new Pose2d(x, y, currentPose.getRotation());
    
            int zoneIndex = noGoZones.whichNoGoZone(probe, bufferDistance);
            if (zoneIndex != -1 && !intersectingZones.contains(zoneIndex)) {
                intersectingZones.add(zoneIndex);
            }
        }
    
        if (intersectingZones.isEmpty()) {
            trajectory = TrajectoryGenerator.generateTrajectory(
                currentPose,
                interiorWaypoints,
                targetPose,
                new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSquared)
            );
        } else {
            // Step 2–3: build detours for each intersecting zone
            for (int zoneIndex : intersectingZones) {
                Pose2d[] bufferedVertices = noGoZones.getZones()[zoneIndex].getBufferedVertices(bufferDistance);
    
                ArrayList<Pose2d> intersections = getLinePolygonIntersections(currentPose, targetPose, bufferedVertices);
                if (intersections.size() == 2) {
                    Pose2d entry = intersections.get(0);
                    Pose2d exit  = intersections.get(1);
    
                    // Build shortest paths clockwise and counterclockwise
                    ArrayList<Translation2d> pathCW  = walkPolygon(entry, exit, bufferedVertices, true);
                    ArrayList<Translation2d> pathCCW = walkPolygon(entry, exit, bufferedVertices, false);
    
                    // Pick shorter path
                    ArrayList<Translation2d> chosen = (pathLength(pathCW) < pathLength(pathCCW)) ? pathCW : pathCCW;
    
                    // Step 4: pick only the waypoint with greatest perpendicular distance from line (currentPose -> targetPose)
                    Translation2d bestWaypoint = null;
                    double maxDistance = -1;
                    for (Translation2d p : chosen) {
                        double d = perpendicularDistance(currentPose, targetPose, p);
                        if (d > maxDistance) {
                            maxDistance = d;
                            bestWaypoint = p;
                        }
                    }
    
                    if (bestWaypoint != null) {
                        interiorWaypoints.add(bestWaypoint);
                    }
                }
            }
    
            trajectory = TrajectoryGenerator.generateTrajectory(
                currentPose,
                interiorWaypoints,
                targetPose,
                new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSquared)
            );
        }
    
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
        // Field relative is true because NoGoZones are field-relative
        swerveDrive.drive(chassisSpeeds, true);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when the command ends.
        swerveDrive.drive(new ChassisSpeeds(0, 0, 0), false);
    }

    @Override
    public boolean isFinished() {
        // The command is finished when the trajectory is complete.
        return timer.get() >= trajectory.getTotalTimeSeconds();
    }
}
