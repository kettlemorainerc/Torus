package org.usfirst.frc.team2077.subsystem.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc.team2077.command.AutoPITuner;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule.MotorPosition;
import org.usfirst.frc.team2077.util.AutoPIable;

public class SwerveDrivingMotor extends AutoPIable {

    private static final double driveGearReduction = (45d * 22d) / (15d * 13d/*This is the variable gear*/);
    private static final int drivingMotorCurrentLimit = 50; // amps

    public static final double wheelRadius = Units.inchesToMeters(2);
    public static final double wheelDiameter = wheelRadius * 2.0;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    private final MotorPosition position;
    private final SwerveModule parent;

//    private final SlewRateLimiter rateLimiter;

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController PID;

    private double velocitySet;
    private boolean reversed = false;

    public SwerveDrivingMotor(MotorPosition position, SwerveModule parent){
        this.parent = parent;
        this.position = position;
//        rateLimiter = new SlewRateLimiter(6.0);

        motor = new CANSparkMax(position.drivingCANid, CANSparkMaxLowLevel.MotorType.kBrushless);
        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        motor.setSmartCurrentLimit(drivingMotorCurrentLimit);

        encoder = motor.getEncoder();
        encoder.setVelocityConversionFactor(wheelCircumference / driveGearReduction / 60.0);

        PID = motor.getPIDController();

        motor.burnFlash();

        init(position.name() + "_DRIVING", 0.00004718000127468258, 0.00031408999348059297);
    }

    public void update(){
        PID.setReference(
//                rateLimiter.calculate(
                (velocitySet) * (reversed? -1 : 1)
//                )
                , CANSparkMax.ControlType.kVelocity
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
        return getVelocityMeasured();
    }

    @Override
    public void tunerSet(double velocity) {
        parent.calibrating = true;

        if(velocity == 0.0){
            motor.set(0.0);
        }else{
            setVelocity(velocity);
            update();
        }

    }

    @Override
    public AutoPITuner.ErrorMethod getErrorMethod() {
        return AutoPITuner.ErrorMethod.DIFFERENCE;
    }
}
