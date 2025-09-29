package frc.robot.resistanceswerve.util;

import edu.wpi.first.math.geometry.Pose2d;

public class NoGoZones {

    /**
     * The NoGoZones on the field.
     */
    NoGoZone[] zones;

    /**
     * Creates a new NoGoZones.
     * This represents all the NoGoZones on a field.
     * @param zones
     */
    public NoGoZones(NoGoZone... zones) {
        this.zones = zones;
    }

    public NoGoZone[] getZones() {
        return zones;
    }

    /**
     * Checks which NoGoZone a given pose is in.
     * If it is within no NoGoZone, returns -1.
     * @return The index of the NoGoZone the pose is in, or -1 if it is in no NoGoZone.
     */
    public int whichNoGoZone(Pose2d pose) {
        for (int i = 0; i < zones.length; i++) {
            if (zones[i].isInNoGoZone(pose)) {
                return i;
            }
        }
        return -1;
    }

    /**
     * Checks which NoGoZone a given pose is in, including a buffer around each NoGoZone.
     * If it is within no buffered NoGoZone, returns -1.
     * @param buffer The buffer distance around each NoGoZone.
     * @return The index of the NoGoZone the pose is in, or -1 if it is in no NoGoZone.
     */
    public int whichNoGoZone(Pose2d pose, double buffer) {
        for (int i = 0; i < zones.length; i++) {
            if (zones[i].isInNoGoZone(pose, buffer)) {
                return i;
            }
        }
        return -1;
    }

    public class NoGoZone {

        Pose2d[] vertices;

        /**
         * Creates a new NoGoZone.
         * A NoGoZone is a polygonal area that the robot should not enter.
         * 
         * @param vertices
         */
        public NoGoZone(Pose2d... vertices) {
            this.vertices = vertices;
        }

        /**
         * Gets the vertices of the NoGoZone.
         * This is useful in creating trajectories around the NoGoZones.
         * @return The vertices of the NoGoZone.
         */
        public Pose2d[] getVertices() {
            return vertices;
        }

        /**
         * Checks if a given pose is inside the NoGoZone.
         * 
         * @param pose The pose to check.
         * @return true if the pose is inside the NoGoZone, false otherwise.
         */
        public boolean isInNoGoZone(Pose2d pose) {
            int intersectCount = 0;
            for (int i = 0; i < vertices.length; i++) {
                Pose2d v1 = vertices[i];
                Pose2d v2 = vertices[(i + 1) % vertices.length];

                // Check if the line from pose to the right intersects with the edge v1-v2
                if (((v1.getY() > pose.getY()) != (v2.getY() > pose.getY())) &&
                        (pose.getX() < (v2.getX() - v1.getX()) * (pose.getY() - v1.getY()) / (v2.getY() - v1.getY())
                                + v1.getX())) {
                    intersectCount++;
                }
            }
            // If the number of intersections is odd, the point is inside the polygon
            return (intersectCount % 2) == 1;
        }

        /**
         * Checks if a given pose is inside a region including the NoGoZone and a buffer around it
         * @param pose The pose to check.
         * @param buffer The buffer distance around the NoGoZone.
         */
        public boolean isInNoGoZone(Pose2d pose, double buffer) {
            // Create a new NoGoZone with the vertices expanded by the buffer distance
            Pose2d[] bufferedVertices = new Pose2d[vertices.length];
            for (int i = 0; i < vertices.length; i++) {
                Pose2d v1 = vertices[i];
                Pose2d v2 = vertices[(i + 1) % vertices.length];

                // Calculate the direction vector from v1 to v2
                double dx = v2.getX() - v1.getX();
                double dy = v2.getY() - v1.getY();
                double length = Math.sqrt(dx * dx + dy * dy);
                if (length == 0) {
                    // If the length is zero, the two vertices are the same point
                    // Just move the vertex outwards by the buffer distance in any direction
                    bufferedVertices[i] = new Pose2d(v1.getX() + buffer, v1.getY(), v1.getRotation());
                } else {
                    // Normalize the direction vector
                    dx /= length;
                    dy /= length;

                    // Calculate the perpendicular vector
                    double px = -dy;
                    double py = dx;

                    // Move the vertex outwards by the buffer distance
                    bufferedVertices[i] = new Pose2d(v1.getX() + px * buffer, v1.getY() + py * buffer, v1.getRotation());
                }
            }

            NoGoZone bufferedZone = new NoGoZone(bufferedVertices);
            return bufferedZone.isInNoGoZone(pose);
        }

        /**
         * Gets the vertices of the NoGoZone expanded by a buffer distance.
         * @param buffer
         * @return The vertices of the buffered NoGoZone.
         */
        public Pose2d[] getBufferedVertices(double buffer) {
            Pose2d[] buffered = new Pose2d[vertices.length];
        
            for (int i = 0; i < vertices.length; i++) {
                Pose2d v1 = vertices[i];
                Pose2d v2 = vertices[(i + 1) % vertices.length];
        
                // Edge vector
                double dx = v2.getX() - v1.getX();
                double dy = v2.getY() - v1.getY();
                double length = Math.hypot(dx, dy);
        
                // Outward normal
                double nx = -dy / length;
                double ny = dx / length;
        
                // Offset current vertex outward
                buffered[i] = new Pose2d(v1.getX() + nx * buffer, v1.getY() + ny * buffer, v1.getRotation());
            }
        
            return buffered;
        }
        
    }
}