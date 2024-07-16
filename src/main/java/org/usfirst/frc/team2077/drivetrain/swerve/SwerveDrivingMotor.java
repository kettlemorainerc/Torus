package org.usfirst.frc.team2077.drivetrain.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import org.usfirst.frc.team2077.util.PIDTuneable;

public class SwerveDrivingMotor implements PIDTuneable {

    private static final int motorFreeSpeed = 5800; //RPM

    private final SwerveConstants.MotorPosition position;
    private final SwerveModule parent;

    private final SlewRateLimiter accelRateLimiter;
    private final SlewRateLimiter deccelRateLimiter;
    private SlewRateLimiter currentRateLimiter;

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController PID;

    private double velocitySet = 0;
    private boolean reversed = false;

    public SwerveDrivingMotor(SwerveConstants.MotorPosition position, SwerveModule parent){
        this.parent = parent;
        this.position = position;

        accelRateLimiter = new SlewRateLimiter(6);
        deccelRateLimiter = new SlewRateLimiter(12);

        motor = new CANSparkMax(position.drivingCANid, CANSparkLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(SwerveConstants.drivingMotorCurrentLimit);

        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(SwerveConstants.wheelCircumference / SwerveConstants.driveGearReduction / 60.0);

        PID = motor.getPIDController();
        PID.setP(position.drivingP);
        PID.setI(position.drivingI);
        PID.setD(0.0);

        motor.burnFlash();
    }

    public void update(){
        if(parent.calibrating){
            return;
        }

//        motor.set(
//                rateLimiter.calculate(
//                        velocitySet
//                ) *  position.drivingF * (reversed? -1 : 1)
//        );

        PID.setReference(
                currentRateLimiter.calculate(
                        velocitySet * (reversed? -1 : 1)
                ),
            CANSparkMax.ControlType.kVelocity
        );
    }

    public double getVelocityMeasured(){
        return encoder.getVelocity();
    }

    public double getVelocitySet() {
        return velocitySet;
    }

    public void setVelocity(double velocity) {
        if(parent.calibrating){
            return;
        }
        //Could we do something here? hmmmm, what could we do? should we look at the methods of SlewRateLimiter?
        if(velocity > Math.abs(getVelocityMeasured())){
            currentRateLimiter = accelRateLimiter;
        }else{
            currentRateLimiter = deccelRateLimiter;
        }
        /*
        Dear Henry
            I write to you from the 2077 robotics room.
         This code was simutaniously nerve racking and simple.
         Why did I do this???

            Dustin
         */

        velocitySet = velocity;
    }

    public boolean getReversed(){
        return reversed;
    }

    public void setReversed(boolean r){
        reversed = r;
    }

    public double getMaximumSpeed(){
        return motorFreeSpeed * encoder.getVelocityConversionFactor();
    }

    public double getP() {
        return PID.getP();
    }
    public double getI() {
        return PID.getI();
    }
    public double getD() {
        return PID.getD();
    }

    public void setP(double p) {
        PID.setP(p);
    }
    public void setI(double i) {
        PID.setI(i);
    }
    public void setD(double d) {
        PID.setD(d);
    }

    @Override
    public void tuningSet(double setpoint) {
        parent.calibrating = true;

        velocitySet = setpoint;

        PID.setReference(
            velocitySet,
            CANSparkMax.ControlType.kVelocity
        );

    }

    @Override
    public void tuningStop() {
        parent.calibrating = true;

        velocitySet = 0.0;

        motor.set(0.0);
    }

    @Override
    public void zeroIntegral() {
        setVelocity(0);
        PID.setIAccum(0.0);
    }

    @Override
    public double tuningGetError() {
        return Math.abs(getVelocityMeasured() - getVelocitySet());
    }

    @Override
    public boolean tuningReady() {
        return Math.abs(getVelocityMeasured()) < 0.01;
    }

    @Override
    public String getName() {
        return position.name() + "_DRIVING_MOTOR";
    }

    public double getDrivingEncoderPosition(){
        return encoder.getPosition();
    }
}
