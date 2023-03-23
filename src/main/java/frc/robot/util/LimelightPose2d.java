package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightPose2d extends Pose2d {
    private final double latency;
    public LimelightPose2d(double x, double y, Rotation2d rotation, double latency) {
        super(x, y, rotation);
        this.latency = latency;
    }

    public LimelightPose2d(Pose2d pose, double latency) {
        super(pose.getX(), pose.getY(), pose.getRotation());
        this.latency = latency;
    }

    public double getLatency() {
        return latency;
    }
}
