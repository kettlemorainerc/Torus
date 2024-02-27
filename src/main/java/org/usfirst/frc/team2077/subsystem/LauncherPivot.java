package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class LauncherPivot implements Subsystem {

    private CANSparkMax motor;
    private SensorCollection encoder;

    private double target = 0.0;
    private static final double deadzone = 1.0;

    private boolean targeting = false;

    private final SmartDashRobotPreference maxSpeed = new SmartDashRobotPreference("Launcher pivot max speed", 0.6);
    private final SmartDashRobotPreference rampRate = new SmartDashRobotPreference("Launcher pivot inverse ramp rate", 35.0);
    private final SmartDashRobotPreference minSpeed = new SmartDashRobotPreference("Launcher pivot min speed", 0.075);

    private final SmartDashNumber displayAngle = new SmartDashNumber("Launcher angle", 0.0, true);

    private static final double zeroOffset = 214.1;

    public LauncherPivot(){
        motor = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushed);
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

        double angle = getAngle();
        double d = Math.signum(p);

        //Softstops:
        if(angle < -45 && d == -1){
            stop();
        }

        if(angle > 120 && d == 1){
            stop();
        }
    }

    public void moveTowardsTarget(){
        double angleDiff = getAngle() - target;
        double dir = -Math.signum(angleDiff);
        double percent = maxSpeed.get();

        System.out.println(angleDiff);

        if(Math.abs(angleDiff) > deadzone){

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

    public void setTarget(double target){
        this.target = target;
        targeting = true;
    }

    public void stop(){
        motor.set(0.0);
        targeting = false;
    }

    public double getAngle() {
        double raw = encoder.getPulseWidthPosition();
        return raw * (360.0 / -4096.0) + zeroOffset;
    }

    public void setTargeting(boolean t){
        targeting = t;
    }

    public boolean atTarget(){
        double angleDiff = SwerveChassis.getAngleDifferenceDegrees(getAngle(), target);
        return Math.abs(angleDiff) < deadzone;
    }

}
