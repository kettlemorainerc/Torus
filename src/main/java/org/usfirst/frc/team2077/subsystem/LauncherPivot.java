package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

import java.sql.SQLOutput;

public class LauncherPivot implements Subsystem {

    private static final double encoderCounts = 4096.0; //Encoder counts
    private static final double deadZone = 1.0; //Degrees

    private static final SmartDashRobotPreference maxSpeed = new SmartDashRobotPreference("Launcher pivot max speed", 0.6); //Percent output
    private static final SmartDashRobotPreference rampRate = new SmartDashRobotPreference("Launcher pivot inverse ramp rate", 35.0); //Percent output
    private static final SmartDashRobotPreference minSpeed = new SmartDashRobotPreference("Launcher pivot min speed", 0.075); //Percent output

    private static final SmartDashNumber displayAngle = new SmartDashNumber("Launcher angle", 0.0, true); //Degrees
    private static final SmartDashNumber displayRaw = new SmartDashNumber("Launcher encoder raw", 0.0, true); //Encoder counts

    private static final int encoderOffset = 1730; //Encoder counts

    private static final double lowerBound =   0.0; //Degrees
    private static final double upperBound = 170.0; //Degrees

    private static final double stallThreshold = 0.0; //Amps TODO: determine stall amperage on the 700:1 gearbox
    private static final double stallTime = 50.0; //Ticks TODO: determine minimum stall time

    private final CANSparkMax motor;
    private final SensorCollection encoder;

    private boolean halted = false;

    private boolean targeting = false;
    private double stallTimer = 0.0; //Ticks
    private double target = 0.0; //Degrees

    public LauncherPivot(){
        motor = new CANSparkMax(15, CANSparkLowLevel.MotorType.kBrushed);
        encoder = new TalonSRX(16).getSensorCollection();

        motor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.register();
    }

    @Override
    public void periodic() {
        displayAngle.set(getAngle());

        if(halted) {
            System.out.println("LauncherPivot is halted");
            stop();
            return;
        }

        if(targeting) moveTowardsTarget();
    }

    public void run(double p){
        motor.set(p);

        double current = motor.getOutputCurrent();
        if(current > stallThreshold){
            //anytime the motor starts moving from being stationary, the current is going to be high
            //we only want to halt the subsystem if this stall continues for an unreasonable amount of time
            //unfortunately this can really only be reliable if the hard stop is strong enough
            stallTimer++;
            if(stallTimer > stallTime){
                System.out.println("LauncherPivot motor has reached stalled");
                System.out.println("Subsystem was automatically halted");
                halted = true;
                return;
            }
        }else{
            stallTimer = 0.0;
        }

        double angle = getAngle();
        if(
            angle < lowerBound - deadZone * 5 ||
            angle > upperBound + deadZone * 5
        ) {
            System.out.println("LauncherPivot has reaching an unreasonable position");
            System.out.println("Subsystem was automatically halted");
            halted = true;
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
        double t = target.angle.get();
        if(t < lowerBound || t > upperBound){
            return;
        }

        this.target = t;
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
