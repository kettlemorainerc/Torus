package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class LauncherPivot implements Subsystem {

    private TalonSRX motor;
    private SensorCollection encoder;

    private double target = 0.0;
    private static final double deadzone = 2.0;

    private boolean targeting = false;

    private final SmartDashNumber speed = new SmartDashNumber("Launcher rotator speed", 0.0, true);
    private final SmartDashNumber displayAngle = new SmartDashNumber("Launcher angle", 0.0, true);

    private static final double zeroOffset = 1760;

    public LauncherPivot(){
        motor = new TalonSRX(14);
        encoder = new TalonSRX(11).getSensorCollection();

        this.register();
    }

    @Override
    public void periodic() {
        displayAngle.set(getAngle());

        if(targeting) moveTowardsTarget();
    }

    public void run(double p){
        motor.set(TalonSRXControlMode.PercentOutput, p);

        double angle = getAngle();
        double dir = Math.signum(p);

        //Softstops:
        if(angle < -45 && dir == -1){
            stop();
        }

        if(angle > 120 && dir == 1){
            stop();
        }
    }

    public void moveTowardsTarget(){
        double angleDiff = SwerveChassis.getAngleDifferenceDegrees(getAngle(), target);
        double p = -Math.signum(angleDiff) * speed.get();

        if(Math.abs(angleDiff) > deadzone){

            if(Math.abs(angleDiff) < 4){
                p *= 0.5;
            }
            //Ideally this is changed to be PID or some other form of feedback control
            run(p);
        }else{
            stop();
        }

    }

    public void setTarget(double target){
        this.target = target;
        targeting = true;
    }

    public void stop(){
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
        targeting = false;
    }

    public double getAngle() {
        double raw = encoder.getPulseWidthPosition();
        double angle = (raw - zeroOffset) * 360.0 / -4096.0;

        return angle;
    }

}
