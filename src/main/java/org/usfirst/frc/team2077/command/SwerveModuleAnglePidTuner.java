package org.usfirst.frc.team2077.command;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import org.usfirst.frc.team2077.subsystem.SwerveModule;
import org.usfirst.frc.team2077.util.SmartDashString;

public class SwerveModuleAnglePidTuner extends PidTuner {
    protected static final double ERROR_MULTIPLIER = 150;

    private final SmartDashString autoPid = new SmartDashString("Auto PID", "", true);

    private final SwerveModule module;
    private final SparkMaxPIDController hardwarePid;
    private final PIDController softwarePid;
    private final AbsoluteEncoder encoder;

    private final double targetAngle;

    public SwerveModuleAnglePidTuner(
          SwerveModule module,
          double initialP, double initialI,
          double target, double duration
    ) {
        super(initialP, initialI, duration);
        this.module = module;
        this.hardwarePid = module.getGuidingPID();
        this.softwarePid = module.getAnglePid();
        this.encoder = module.getAngleEncoder();

        this.targetAngle = target;
    }

    @Override protected void test(double changeInTime) {
        double angleDifference = SwerveModule.getAngleDifference(targetAngle, getAngle());

        currentError += Math.abs(changeInTime * angleDifference * angleDifference) * ERROR_MULTIPLIER;

        setAngle(targetAngle);
    }

    @Override protected void stopTest() {
        setAngle(0);
    }

    @Override protected boolean isTestStopped() {
        return Math.abs(SwerveModule.getAngleDifference(getAngle(), 0)) < 0.25D;
    }

    @Override public void end(boolean interrupted) {
        setPid(bestP, bestI);

        module.getGuidingMotor().burnFlash();
    }

    private double getAngle() {
        return module.getAngle();
    }

    private void setAngle(double angle) {
        module.setAngle(angle);
    }

    private void setPid(double p, double i) {
        hardwarePid.setP(p);
        hardwarePid.setI(i);

        softwarePid.setP(p);
        softwarePid.setI(i);
    }
}
