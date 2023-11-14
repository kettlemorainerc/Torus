package org.usfirst.frc.team2077.math;

public class SwerveTargetValues {
    private double magnitude, angle;

    public SwerveTargetValues(double magnitude, double angle) {
        this.magnitude = magnitude;
        this.angle = angle;
    }

    public double getMagnitude() {
        return magnitude;
    }

    public double getAngle() {
        return angle;
    }

    public void setMagnitude(double magnitude) {
        this.magnitude = magnitude;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    @Override public String toString() {
        return String.format("[mag=%s][ang=%s]", magnitude, angle);
    }
}
