package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimelightPose2d extends Pose2d {
    public final double latency;
    public final int aprilTagAmount;
    public LimelightPose2d(double x, double y, Rotation2d rotation, double latency, int aprilTagAmount) {
        super(x, y, rotation);
        this.latency = latency;
        this.aprilTagAmount = aprilTagAmount;
    }

    public LimelightPose2d(Pose2d pose, double latency, int aprilTagAmount) {
        super(pose.getX(), pose.getY(), pose.getRotation());
        this.latency = latency;
        this.aprilTagAmount = aprilTagAmount;
    }
}
