package org.usfirst.frc.team2077.subsystem.swerve;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.controller.PIDController;
import org.usfirst.frc.team2077.command.AutoPITuner;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule.MotorPosition;
import org.usfirst.frc.team2077.util.AutoPIable;

public class SwerveGuidingMotor extends AutoPIable {

    private static final int guidingMotorCurrentLimit = 20; // amps

    private final SwerveModule.MotorPosition position;
    private final SwerveModule parent;

    private final CANSparkMax motor;
    private final AbsoluteEncoder encoder;

    private final PIDController PID;

    private double angleOffset = 0.0;
    private double angleSet = 0.0;

    public SwerveGuidingMotor(SwerveModule.MotorPosition position, SwerveModule parent) {

        this.parent = parent;
        this.position = position;

        angleOffset = position.angleOffset;

        motor = new CANSparkMax(position.guidingCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(guidingMotorCurrentLimit);

        encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder.setPositionConversionFactor(2.0 * Math.PI);
        encoder.setInverted(false);

        PID = new PIDController(0.0, 0.0, 0.0);//TODO: FIX
        // PID.getP(), guidingCANPID.getI(), 0.0);

        motor.burnFlash();

        init(position.name() + "_GUIDING", 0.11240000277757645, 0.000039149999793153256);
    }

    public void update(){
        if(!parent.calibrating && Math.abs(parent.getDrivingMotor().getVelocitySet()) < 0.01) {
            motor.set(0.0);
            return;
        }

        double angleDiff = SwerveChassis.getAngleDifference(angleSet, getAngle());
        double p = PID.calculate(Math.abs(angleDiff), 0.0) * Math.signum(angleDiff);
//        System.out.println(PID.getP());
        motor.set(p);
    }

    public double getAngle() {
        double angle = encoder.getPosition() + angleOffset;
        angle %= 2.0 * Math.PI;
        if(angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    public void setAngle(double angle) {
        if(parent.calibrating){
            return;
        }

        double currentAngle = getAngle();
        double angleDifference = SwerveChassis.getAngleDifference(currentAngle, angle);

        SwerveDrivingMotor drivingMotor = parent.getDrivingMotor();
        boolean reversed = drivingMotor.getReversed();
        double velocitySet = drivingMotor.getVelocitySet();

        if (Math.abs(velocitySet) > 0.5) {
            if (reversed) {
                angle -= Math.PI;
            }
        } else if (Math.abs(angleDifference) > 0.5 * Math.PI) {
            angle -= Math.PI;
            reversed = true;
        } else {
            reversed = false;
        }

        drivingMotor.setReversed(reversed);

        angle %= 2.0 * Math.PI;
        if (angle < 0) angle += 2.0 * Math.PI;
        angleSet = angle;
    }

    @Override
    public double getP() {
        return PID.getP();
    }

    @Override
    public double getI() {
        return PID.getI();
    }

    @Override
    public void setP(double p) {
        PID.setP(p);
    }

    @Override
    public void setI(double i) {
        PID.setI(i);
    }

    @Override
    public double tunerGet() {
        return getAngle();
    }

    @Override
    public void tunerSet(double angle) {
        parent.calibrating = true;

        if (angle == 0.0) {
            angleOffset -= getAngle();
            motor.set(0.0);
            return;
        }

        angle %= 2.0 * Math.PI;
        if (angle < 0) angle += 2.0 * Math.PI;
        angleSet = angle;

        double angleDiff = SwerveChassis.getAngleDifference(angleSet, getAngle());
        double p = PID.calculate(Math.abs(angleDiff), 0.0) * Math.signum(angleDiff);
        System.out.println(p);
        motor.set(p);
    }

    @Override
    public AutoPITuner.ErrorMethod getErrorMethod() {
        return AutoPITuner.ErrorMethod.ANGLE_DIFFERENCE;
    }
}
