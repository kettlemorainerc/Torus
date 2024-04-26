package org.usfirst.frc.team2077.subsystem;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.PIDTuneable;
import org.usfirst.frc.team2077.util.SmartDash.SmartDashRobotPreference;

public class Launcher implements Subsystem {

    public enum Target{
        INTAKE(-5, 0),
        AMP(10, 140),
        SPEAKER(25, 90),
        STAGE(10, 130);
        public final SmartDashRobotPreference speed, angle;
        Target(double defaultSpeed, double defaultAngle){
            speed = new SmartDashRobotPreference(String.format("Launcher %s speed", this.name()), defaultSpeed);
            angle = new SmartDashRobotPreference(String.format("Launcher %s angle", this.name()), defaultAngle);
        }
    }

    public final LauncherMotor launcherMotorLeft, launcherMotorRight;
    private final CANSparkMax feederMotorLeft, feederMotorRight;

    private final SmartDashRobotPreference feederIntakeSpeed = new SmartDashRobotPreference("feeder intake percent", 0.2);
    private final SmartDashRobotPreference feederSpeed = new SmartDashRobotPreference("feeder feed percent", 1.0);

    private double launcherSpeedSet = 0.0;
//    private SlewRateLimiter limiter = new SlewRateLimiter(0.02 / 0.002);

    public Launcher(){
        launcherMotorLeft = new LauncherMotor(11,  0.0001, 0.00001);
        launcherMotorRight = new LauncherMotor(12, 0.0001, 0.00001);

        feederMotorLeft = new CANSparkMax(13, CANSparkLowLevel.MotorType.kBrushed);
        feederMotorRight = new CANSparkMax(14, CANSparkLowLevel.MotorType.kBrushed);

        feederMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        feederMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        this.register();
    }

    public void periodic(){
        launcherMotorLeft.run(-launcherSpeedSet);
        launcherMotorRight.run(launcherSpeedSet);
    }

    public void run(Target target){
        launcherSpeedSet = target.speed.get();

        if(target == Target.INTAKE){
            feederMotorLeft.set(-feederIntakeSpeed.get());
            feederMotorRight.set(feederIntakeSpeed.get());
        }
    }

    public boolean atSpeed(){
        return launcherMotorLeft.atSpeed() && launcherMotorRight.atSpeed();
    }

    public void feed(){
        if(!atSpeed() || launcherSpeedSet < 1){
            stopFeed();
            return;
        }

//        double r = limiter.calculate(feederSpeed.get());
        double r = feederSpeed.get();

        feederMotorLeft.set(r);
        feederMotorRight.set(-r);
    }

    public void stopFeed(){
//        limiter.reset(0.0);
        feederMotorLeft.set(0.0);
        feederMotorRight.set(0.0);
    }

    public void stopLauncher(){
        launcherSpeedSet = 0.0;
    }

    public class LauncherMotor implements PIDTuneable {

        private final double atSpeedTheshold = 10.0; //RPM
        private boolean calibrating = false;

        private final CANSparkMax motor;
        private final RelativeEncoder encoder;
        private final SparkPIDController PID;

        private double target = 0.0;

        public LauncherMotor(int id, double p, double i){
            motor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);

            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);

            encoder = motor.getEncoder();

//            encoder.setVelocityConversionFactor(wheelCircumference / 60.0);

            PID = motor.getPIDController();
            PID.setP(p);
            PID.setI(i);
            PID.setD(0.0);
        }

        public void run(double speed){
            if(calibrating) return;

            target = speed;

            if(Math.abs(speed) < 0.05){
                motor.set(0.0);
                return;
            }

            PID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        }

        public boolean atSpeed(){
            return Math.abs(encoder.getVelocity() - target) < atSpeedTheshold;
        }

        public double getP() { return PID.getP(); }
        public double getI() { return PID.getI(); }
        public double getD() { return PID.getD(); }

        public void setP(double p) { PID.setP(p); }
        public void setI(double i) { PID.setI(i); }
        public void setD(double d) { PID.setD(d); }

        @Override
        public void tuningSet(double setpoint) {
            calibrating = true;
            PID.setReference(setpoint, CANSparkMax.ControlType.kVelocity);
        }

        @Override
        public void tuningStop() {
            calibrating = true;
            motor.set(0.0);
        }

        @Override
        public void zeroIntegral() {
            PID.setIAccum(0.0);
        }

        @Override
        public double tuningGetError() {
            return Math.abs(encoder.getVelocity() - target);
        }

        @Override
        public boolean tuningReady() {
            return Math.abs(encoder.getVelocity()) < 0.1;
        }
    }
}
