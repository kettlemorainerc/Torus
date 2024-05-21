package org.usfirst.frc.team2077.subsystem.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.PIDTuneable;
import org.usfirst.frc.team2077.util.SmartDash.SmartDashNumber;

public class SwerveGuidingMotor implements PIDTuneable {

    private static final int guidingMotorCurrentLimit = 20; // amps

    private final SwerveModule.MotorPosition position;
    private final SwerveModule parent;

    private final CANSparkMax motor;

    private final AbsoluteEncoder absoluteEncoder;
    private final RelativeEncoder relativeEncoder;

    private  final PIDController PID;

    private double angleOffset = 0.0;
    private double angleSet = 0.0;

    private boolean zeroVelocity = false;

    private double atAngleDeadzone = Math.PI / 12.0;

    private static SmartDashNumber p = new SmartDashNumber("Guiding P", 0.0, true);
    private static SmartDashNumber v = new SmartDashNumber("Guiding V", 0.0, true);

    private static final double azimuthRatio = 203d / 9424d;

    public SwerveGuidingMotor(SwerveModule.MotorPosition position, SwerveModule parent) {
        this.parent = parent;
        this.position = position;

        angleOffset = position.angleOffset;

        motor = new CANSparkMax(position.guidingCANid, CANSparkLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
        motor.setSmartCurrentLimit(guidingMotorCurrentLimit);

        absoluteEncoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
        absoluteEncoder.setPositionConversionFactor(2.0 * Math.PI);
        absoluteEncoder.setVelocityConversionFactor(2.0 * Math.PI);
        absoluteEncoder.setInverted(false);

        relativeEncoder = motor.getEncoder();
        relativeEncoder.setPositionConversionFactor(2.0 * Math.PI * azimuthRatio);
        relativeEncoder.setVelocityConversionFactor(2.0 * Math.PI * azimuthRatio);
//        relativeEncoder.setInverted(true);


        relativeEncoder.setPosition(0.0);
        angleOffset += absoluteEncoder.getPosition();

        PID = new PIDController(position.guidingP, position.guidingI, 0.0);

        motor.burnFlash();
    }

    public void update(){
        if(parent.calibrating) {
            motor.set(0.0);
            return;
        }

        double p = distanceToTarget();
        double v = relativeEncoder.getVelocity();

//        double c = p * position.guidingP - v * position.guidingI;

        double c = p * SwerveGuidingMotor.p.get() - v * SwerveGuidingMotor.v.get();


//        double c = PID.calculate(Math.abs(p), 0.0) * Math.signum(p);
//d
//        if(Math.abs(p) < 0.001){
//            p = 0.0;
//        }

        motor.set(c);
    }

    public double getAngle() {
        double angle = Math.PI * 2 - relativeEncoder.getPosition();

        angle = angle + angleOffset;
        angle %= 2.0 * Math.PI;
        if(angle < 0) angle += 2.0 * Math.PI;
        return angle;
    }

    public boolean atAngle(){
        return Math.abs(distanceToTarget()) <= atAngleDeadzone;
    }

    public void setAngle(double angle) {
        if(parent.calibrating){
            return;
        }

//        double angleDifference = Math.abs(SwerveChassis.getAngleDifference(angle, getAngle()));
////
////        SwerveDrivingMotor drivingMotor = parent.getDrivingMotor();
////        boolean reversed = drivingMotor.getReversed();
////        double velocitySet = Math.abs(drivingMotor.getVelocitySet());
//////
//////        System.out.println(zeroVelocity);
//////
////        if (!zeroVelocity) {
////            if (reversed) {
////                angle -= Math.PI;
////            }
////        } else if (angleDifference > 0.5 * Math.PI) {
////            angle -= Math.PI;
////            reversed = true;
////        } else {
////            reversed = false;
////        }
////
////
////        drivingMotor.setReversed(reversed);
////
////        zeroVelocity = velocitySet < 0.1;
////
////        if(zeroVelocity){
////            return;
////        }

        angle %= 2.0 * Math.PI;
        if (angle < 0) angle += 2.0 * Math.PI;
        angleSet = angle;
    }

    //Use sparingly (duh)
    public void setAngleForced(double angle){
        double angleDifference = distanceToTarget();

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

    public double distanceToTarget(){
        return SwerveChassis.getAngleDifference(angleSet, getAngle());
    }

    public double getP(){ return PID.getP(); }
    public double getI(){ return PID.getI(); }
    public double getD(){ return PID.getD(); }

    public void setP(double p) { PID.setP(p); }
    public void setI(double i) { PID.setI(i); }
    public void setD(double d) { PID.setD(d); }

    private double setpoint = 0;

    @Override
    public void tuningSet(double setpoint) {
        parent.calibrating = true;

        this.setpoint = setpoint;
        double p = SwerveChassis.getAngleDifference(setpoint, getAngle());
        double v = absoluteEncoder.getVelocity();

//        double c = p * position.guidingP - v * position.guidingI;

        double c = p * PID.getP() - v * PID.getI();

//        double p = PID.calculate(Math.abs(angleDiff), 0.0) * Math.signum(angleDiff);
//
//        if(Math.abs(p) < 0.001){
//            p = 0.0;
//        }

        motor.set(c);
    }

    @Override
    public void tuningStop() {
        parent.calibrating = true;

        motor.set(0.0);
    }

    @Override
    public void zeroIntegral() {
        angleOffset -= getAngle();
        PID.reset();
    }

    @Override
    public double tuningGetError() {
        return Math.abs(SwerveChassis.getAngleDifference(setpoint, getAngle()));
    }

    @Override
    public boolean tuningReady() {
        return Math.abs(motor.getEncoder().getVelocity()) < 0.01;
    }

    @Override
    public String getName() {
        return position.name() + "_GUIDING_MOTOR";
    }
}
