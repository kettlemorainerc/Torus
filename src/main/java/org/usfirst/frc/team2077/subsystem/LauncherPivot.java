package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;
import org.usfirst.frc.team2077.util.SmartDashString;

public class LauncherPivot implements Subsystem {

    private enum State{
        RUNNING,
        CALIBRATING,
        HALTED
    }

    private static final double deadZone = 1.0; //Degrees
    private static final int encoderCounts = 4096; //Encoder counts

    private static final SmartDashRobotPreference maxSpeed = new SmartDashRobotPreference("Launcher pivot max speed", 0.6); //Percent output
    private static final SmartDashRobotPreference rampRate = new SmartDashRobotPreference("Launcher pivot inverse ramp rate", 35.0); //Percent output
    private static final SmartDashRobotPreference minSpeed = new SmartDashRobotPreference("Launcher pivot min speed", 0.075); //Percent output

    private static final SmartDashNumber displayAngle = new SmartDashNumber("Launcher angle", 0.0, false); //Degrees
    private static final SmartDashNumber displayRaw = new SmartDashNumber("Launcher encoder raw", 0.0, false); //Encoder counts

    private static final SmartDashString statusDebug = new SmartDashString("Pivot status", "", false);

    private static final double upperBound = 170.0; //Degrees
    private static final double lowerBound =   0.0; //Degrees

    private static int encoderOffset = 1743; //Encoder counts

    private static final double stallTime = 10.0; //Ticks TODO: determine minimum stall time

    private final CANSparkMax motor;
    private final SensorCollection encoder;

    private State state = State.RUNNING;

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

        statusDebug.set(String.format("Launcher is %s", state.name()));

        switch(state){
            case RUNNING:
                if(targeting) moveTowardsTarget();
                return;
            case CALIBRATING:
                calibrate();
                return;
            case HALTED:
                motor.set(0.0);
        }

    }

    /**
     * Slowly moves the pivot down until it reaches the lower hardstop
     */
    private void calibrate(){
        motor.set(0.1);

        double v = encoder.getPulseWidthVelocity();
        double x = 1.0;
        //If the encoder is moving less than 1 degree per second, then it is considered stalling
        if(Math.abs(v) < x * 360.0 / encoderCounts ) {
            stallTimer++;
            if(stallTimer > stallTime){
                encoderOffset = encoder.getPulseWidthPosition() % encoderCounts;
                if(encoderOffset < 0.0) encoderOffset += encoderCounts;

                state = State.RUNNING;
            }
        }else{
            stallTimer = 0.0;
        }
    }

    public void run(double p){
        motor.set(p);

//        double v = encoder.getPulseWidthVelocity();
//        double x = 1;s
//        //If the encoder is moving less than 1 degree per second, then it is considered stalling
//        if(Math.abs(v) < x * 360.0 / encoderCounts && Math.abs(p) > minSpeed.get() * 1.25 ){
//            stallTimer++;
//            if(stallTimer > stallTime){
//                System.out.println("LauncherPivot motor has reached stalled");
//                System.out.println("Subsystem was automatically halted");
//                state = State.HALTED;
//                return;
//            }
//        }else{
//            stallTimer = 0.0;
//        }

        double angle = getAngle();
        if(
            angle < lowerBound - deadZone * 5 ||
            angle > upperBound + deadZone * 5
        ) {
            System.out.println("LauncherPivot has reaching an unreasonable position");
            System.out.println("Subsystem was automatically halted");
            state = State.HALTED;
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

    /**
     * Begins auto-calibrating the pivot
     */
     public void reset(){
//         if(state != State.HALTED) return;
         state = State.CALIBRATING;
    }

}
