package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.MathUtils;

public class SmoothingFilter {
    private final double THRESHOLD = 0.9;
    private final double FREQUENCY = 20.0; // how often the thing's updated I think??
    private boolean ready = false;
    private double timerInterval = 1000.0 / FREQUENCY;
    private double latency = 25.0;
    private double strength = 3.0;
    private double multiplier = 1.0;
    private double smoothingOffsetX = 0.0;
    private double smoothingOffsetY = 1.0;
    private double weight;
    private Pose2d position;

    public Pose2d filter(Pose2d calcTarget) {
        if (!ready) {
            position = calcTarget;
            setWeight(latency);
            ready = true;
            return calcTarget;
        }

        var delta = new Pose2d(calcTarget.getX() - position.getX(), calcTarget.getY() - position.getY(),
                new Rotation2d(calcTarget.getRotation().getRadians() - position.getRotation().getRadians()));
        var distance = Math.hypot(calcTarget.getX() - position.getX(), calcTarget.getY() - position.getY());

        // Someone tell me how and why this works.
        var weightModifier = Math.pow(distance + smoothingOffsetX, strength * -1) * multiplier;

        // Limit minimum
        if (weightModifier + smoothingOffsetY < 0) {
            weightModifier = 0;
        } else {
            weightModifier += smoothingOffsetY;
        }

        weightModifier = weight / weightModifier;
        weightModifier = MathUtils.clamp(weightModifier, 0, 1);
        position = new Pose2d(position.getX() + delta.getX() * weightModifier,
            position.getY() + delta.getY() * weightModifier, new Rotation2d(
                position.getRotation().getRadians() + delta.getRotation().getRadians() * weightModifier));
        
        return position;
    }

    private void setWeight(double latency) {
        double stepCount = latency / timerInterval;
        double target = 1 - THRESHOLD;
        weight = 1.0 - (1.0 / Math.pow(1.0 / target, 1.0 / stepCount));
    }
}
