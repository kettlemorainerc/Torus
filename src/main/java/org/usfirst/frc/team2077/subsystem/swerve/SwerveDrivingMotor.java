package org.usfirst.frc.team2077.subsystem.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule.MotorPosition;
import org.usfirst.frc.team2077.util.PIDTuneable;

public class SwerveDrivingMotor implements PIDTuneable {

    private static final int drivingMotorCurrentLimit = 50; // amps
    private static final int motorFreeSpeed = 5800; //RPM

    public static final double wheelDiameter = Units.inchesToMeters(2.9);
    public static final double wheelRadius = 0.5 * wheelDiameter;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    private static final double driveGearReduction = (45d * 22d) / (15d * 13d/*This is the variable gear*/);

    private final MotorPosition position;
    private final SwerveModule parent;

    private final SlewRateLimiter rateLimiter;

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController PID;

    private double velocitySet;
    private boolean reversed = false;

    public SwerveDrivingMotor(MotorPosition position, SwerveModule parent){
        this.parent = parent;
        this.position = position;
        rateLimiter = new SlewRateLimiter(10.0);

        motor = new CANSparkMax(position.drivingCANid, CANSparkLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(drivingMotorCurrentLimit);

        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(wheelCircumference / driveGearReduction / 60.0);

        PID = motor.getPIDController();
        PID.setP(position.drivingP);
        PID.setI(position.drivingI);
        PID.setD(0.0);

        motor.burnFlash();
    }

    public void update(){
        if(
            parent.calibrating ||
            (
                Math.abs(velocitySet) < 0.05 && (
                    RobotHardware.getInstance().getChassis().mode == SwerveChassis.DriveMode.BRAKE ||
                    Math.abs(getVelocityMeasured()) < 0.01
                )
            ) || position != SwerveModule.MotorPosition.BACK_LEFT
        ){
            motor.set(0.0);
            rateLimiter.reset(0.0);
            return;
        }

        PID.setReference(
            rateLimiter.calculate(
                (velocitySet) * (reversed? -1 : 1)
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

        PID.setReference(
            velocitySet,
            CANSparkMax.ControlType.kVelocity
        );

    }

    @Override
    public void tuningStop() {
        parent.calibrating = true;

        motor.set(0.0);
    }

    @Override
    public void zeroIntegral() {
        PID.setIAccum(0.0);
    }

    @Override
    public double tuningGetError() {
        return Math.abs(getVelocityMeasured() - getVelocitySet());
    }

    @Override
    public boolean tuningReady() {
        return Math.abs(getVelocityMeasured()) < 0.001;
    }

    @Override
    public String getName() {
        return position.name() + "_DRIVING_MOTOR";
    }

    public double getDrivingEncoderPosition(){
        return encoder.getPosition();
    }
}
