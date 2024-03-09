package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class LauncherPivot implements Subsystem {

    private CANSparkMax motor;
    private SensorCollection encoder;

    private static final double encoderCounts = 4096.0;

    private double target = 0.0;
    private static final double deadZone = 1.0;

    private boolean targeting = false;

    private final SmartDashRobotPreference maxSpeed = new SmartDashRobotPreference("Launcher pivot max speed", 0.6);
    private final SmartDashRobotPreference rampRate = new SmartDashRobotPreference("Launcher pivot inverse ramp rate", 35.0);
    private final SmartDashRobotPreference minSpeed = new SmartDashRobotPreference("Launcher pivot min speed", 0.075);

    private final SmartDashNumber displayAngle = new SmartDashNumber("Launcher angle", 0.0, true);
    private final SmartDashNumber displayRaw = new SmartDashNumber("Launcher encoder raw", 0.0, true);

    private static final double encoderOffset = 1730;

    public LauncherPivot(){
        motor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushed);
        encoder = new TalonSRX(16).getSensorCollection();

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.register();
    }

    @Override
    public void periodic() {
        displayAngle.set(getAngle());

        if(targeting) moveTowardsTarget();
    }

    public void run(double p){
        motor.set(p);

//        double v = encoder.getPulseWidthVelocity();
//        Encoder velocity opposes the direction of the motor
//        if(Math.signum(v) == Math.signum(p)){
//            stop();
//            return;
//        }

        double angle = getAngle();
        double d = Math.signum(p);

        //Softstops:
        if(angle < 0 && d == -1){
            stop();
            return;
        }

        if(angle > 120 && d == 1){
            stop();
        }
    }

    public void moveTowardsTarget(){
        double angleDiff = getAngle() - target;
        double dir = Math.signum(angleDiff);
        double percent = maxSpeed.get();

        if(Math.abs(angleDiff) > deadZone){

            //Ideally this is changed to be PID or some other form of feedback control
            double rate = rampRate.get();
            if(rate == 0){
                rate = 1;
            }
            percent *= (Math.abs(angleDiff) / rate);

            if(percent < minSpeed.get()) percent = minSpeed.get();
            if(percent > maxSpeed.get()) percent = maxSpeed.get();

            run(percent * dir);
        }else{
            stop();
        }

    }

    public void setTarget(Launcher.Target target){
        this.target = target.angle.get();
        targeting = true;
    }

    public void stop(){
        motor.set(0.0);
        targeting = false;
    }

    public double getAngle() {
        double raw = encoder.getPulseWidthPosition();
        displayRaw.set(raw);

        raw = (raw - encoderOffset) % encoderCounts;
        if(raw < 0) raw += encoderCounts;

        double angle = raw * (360.0 / encoderCounts);
        //Sets the angle to a range from (0-270) U (0, -90)
        if(angle > 270) angle -= 360;

        return angle;
    }

    public void setTargeting(boolean t){
        targeting = t;
    }

    public boolean atTarget(){
        double angleDiff = SwerveChassis.getAngleDifferenceDegrees(getAngle(), target);
        return Math.abs(angleDiff) < deadZone;
    }

}
