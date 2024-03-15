package org.usfirst.frc.team2077.subsystem.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import org.usfirst.frc.team2077.command.AutoPITuner;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
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

    private boolean zeroVelocity = false;


    private double atAngleDeadzone = Math.PI / 12.0;


    public SwerveGuidingMotor(SwerveModule.MotorPosition position, SwerveModule parent) {

        this.parent = parent;
        this.position = position;

        angleOffset = position.angleOffset;

        motor = new CANSparkMax(position.guidingCANid, CANSparkLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(guidingMotorCurrentLimit);

//        Funny spot the difference
//        motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
        encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

        encoder.setPositionConversionFactor(2.0 * Math.PI);
        encoder.setInverted(false);

        PID = new PIDController(0.0, 0.0, 0.0);//TODO: FIX
        // PID.getP(), guidingCANPID.getI(), 0.0);

        motor.burnFlash();

        init(position.name() + "_GUIDING", position.guidingP, position.guidingI, true);
    }

    public void update(){
        if(parent.calibrating) {
            motor.set(0.0);
            return;
        }

        double angleDiff = SwerveChassis.getAngleDifference(angleSet, getAngle());
        double p = PID.calculate(Math.abs(angleDiff), 0.0) * Math.signum(angleDiff);

        if(Math.abs(p) < 0.0001){
            p = 0.0;
        }
//        System.out.println(PID.getP());
        motor.set(p);
    }

    public double getAngle() {
        double angle = encoder.getPosition() + angleOffset;
        angle %= 2.0 * Math.PI;
        if(angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    public boolean atAngle(){
        return Math.abs(SwerveChassis.getAngleDifference(angleSet, getAngle())) <= atAngleDeadzone;
    }

    public void setAngle(double angle) {
        if(parent.calibrating){
            return;
        }

        double currentAngle = getAngle();
        double angleDifference = SwerveChassis.getAngleDifference(currentAngle, angle);

        SwerveDrivingMotor drivingMotor = parent.getDrivingMotor();
        boolean reversed = drivingMotor.getReversed();
        double velocitySet = Math.abs(drivingMotor.getVelocitySet());

        if (!zeroVelocity) {
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
        zeroVelocity = velocitySet < 0.01;

        if(velocitySet < 0.01){
            return;
        }

        angle %= 2.0 * Math.PI;
        if (angle < 0) angle += 2.0 * Math.PI;
        angleSet = angle;
    }

    //Use sparingly (duh)
    public void setAngleForced(double angle){
        double currentAngle = getAngle();
        double angleDifference = SwerveChassis.getAngleDifference(currentAngle, angle);

        SwerveDrivingMotor drivingMotor = parent.getDrivingMotor();

        if (Math.abs(angleDifference) > 0.5 * Math.PI) {
            angle -= Math.PI;
            drivingMotor.setReversed(true);
        } else {
            drivingMotor.setReversed(false);
        }

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
        motor.set(p);
    }

    @Override
    public AutoPITuner.ErrorMethod getErrorMethod() {
        return AutoPITuner.ErrorMethod.ANGLE_DIFFERENCE;
    }


}
