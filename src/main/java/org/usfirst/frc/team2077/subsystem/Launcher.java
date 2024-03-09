package org.usfirst.frc.team2077.subsystem;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.command.AutoPITuner;
import org.usfirst.frc.team2077.util.AutoPIable;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class Launcher implements Subsystem {

    public enum Target{
        INTAKE(-5, 0),
        AMP(10, 120),
        SPEAKER(15, 90);
        public SmartDashRobotPreference speed, angle;
        Target(double defaultSpeed, double defaultAngle){
            speed = new SmartDashRobotPreference(String.format("Launcher %s speed", this.name()), defaultSpeed);
            angle = new SmartDashRobotPreference(String.format("Launcher %s angle", this.name()), defaultAngle);
        }
    }

    public final LauncherMotor launcherMotorLeft, launcherMotorRight;
    private final CANSparkMax feederMotorLeft, feederMotorRight;

    private final SmartDashRobotPreference feederSpeed = new SmartDashRobotPreference("feeder feed percent", 1.0);

    private final SmartDashRobotPreference feederIntakeSpeed = new SmartDashRobotPreference("feeder intake percent", 0.2);

    private double launcherSpeedSet = 0.0;

    public Launcher(){
        launcherMotorLeft = new LauncherMotor(11, "LEFT_LAUNCHER");
        launcherMotorRight = new LauncherMotor(12, "RIGHT_LAUNCHER");

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
        return launcherMotorLeft.atSpeed() && !launcherMotorRight.atSpeed();
    }

    public void feed(){
        if(!atSpeed()){
            return;
        }

        feederMotorLeft.set(feederSpeed.get());
        feederMotorRight.set(-feederSpeed.get());
    }

    public void stopFeed(){
        feederMotorLeft.set(0.0);
        feederMotorRight.set(0.0);
    }

    public void stopLauncher(){
        launcherSpeedSet = 0.0;
    }

    public class LauncherMotor extends AutoPIable{

        private final double wheelRadius = Units.inchesToMeters(2);
        private final double wheelCircumference = wheelRadius * 2.0 * Math.PI;

        private final CANSparkMax motor;
        private final RelativeEncoder encoder;
        private final SparkPIDController PID;

        private double speedUpDeadZone = 1.0;
        private double target = 0.0;

        private boolean calibrating = false;

        public LauncherMotor(int id, String key){
            motor = new CANSparkMax(id, CANSparkLowLevel.MotorType.kBrushless);

            motor.setIdleMode(CANSparkMax.IdleMode.kCoast);

            encoder = motor.getEncoder();

            encoder.setVelocityConversionFactor(wheelCircumference / 60.0);

            PID = motor.getPIDController();


            this.init(key, 0.0000006847124041087227, 0.00002554714410507586, false);
        }

        public void run(double speed){
            if(calibrating) return;

            target = speed;

            if(Math.abs(speed) < 0.01){
                motor.set(0.0);
                return;
            }

            PID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        }

        public boolean atSpeed(){
            return Math.abs(encoder.getVelocity() - target) < speedUpDeadZone;
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
            return encoder.getVelocity();
        }

        @Override
        public void tunerSet(double speed) {
            calibrating = true;

            if(Math.abs(speed) < 0.01){
                motor.set(0.0);
                return;
            }

            PID.setReference(speed, CANSparkMax.ControlType.kVelocity);
        }

        @Override
        public AutoPITuner.ErrorMethod getErrorMethod() {
            return AutoPITuner.ErrorMethod.DIFFERENCE;
        }
    }
}
