package org.usfirst.frc.team2077.command;

import com.revrobotics.*;

public class SparkMotorControllerPidTuner extends PidTuner {
    protected static final double ERROR_MULTIPLIER = 10;

    // Motor being worked on
    private final BetterCanSparkMax controller;
    private final SparkMaxPIDController pid;

    private final double target;

    public SparkMotorControllerPidTuner(
          BetterCanSparkMax controller,
          double initialP, double initialI,
          double target, double duration
    ) {
        super(initialP, initialI, duration);

        this.controller = controller;
        this.pid = controller.getPIDController();
        this.target = target;
    }

    @Override protected void test(double changeInTime) {
        double currentVelocity = getVelocity();

        double offBy = target - currentVelocity;
        currentError += Math.abs(changeInTime * offBy * offBy) * ERROR_MULTIPLIER;

        setVelocity(target);
    }

    @Override public void end(boolean interrupted) {
        SparkMaxPIDController controller = this.controller.getPIDController();

        controller.setP(bestP);
        controller.setI(bestI);
        this.controller.burnFlash();
    }

    protected final double getVelocity() {
        return controller.getEncoder().getVelocity();
    }

    protected final void setVelocity(double target) {
        controller.setTargetVelocity(target);
    }

    @Override protected void stopTest() {
        setVelocity(0);
    }

    @Override protected boolean isTestStopped() {
        return getVelocity() < 0.01D;
    }
}
