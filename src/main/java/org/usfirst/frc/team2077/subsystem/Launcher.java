package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class Launcher implements Subsystem {
    private CANSparkMax launcherMotorLeft;
    private CANSparkMax launcherMotorRight;

    private TalonSRX feederMotorLeft;
    private TalonSRX feederMotorRight;

    private SmartDashNumber launcherSpeedFast = new SmartDashNumber("fast launcher launch percent", 0.0, true);
    private SmartDashNumber launcherSpeedSlow = new SmartDashNumber("slow launcher launch percent", 0.0, true);
    private SmartDashNumber feederSpeed = new SmartDashNumber("feeder feed percent", 0.0, true);

    private SmartDashNumber launcherIntakeSpeed = new SmartDashNumber("launcher intake percent", 0.0, true);
    private SmartDashNumber feederIntakeSpeed = new SmartDashNumber("feeder intake percent", 0.0, true);

    public Launcher(){
        launcherMotorLeft = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushless);
        launcherMotorRight = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushless);

        launcherMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        launcherMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        feederMotorLeft = new TalonSRX(11);
        feederMotorRight = new TalonSRX(12);
    }

    public void runLauncherFast(){
        launcherMotorLeft.set(launcherSpeedFast.get());
        launcherMotorRight.set(-launcherSpeedFast.get());
    }

    public void runLauncherSlow(){
        launcherMotorLeft.set(launcherSpeedSlow.get());
        launcherMotorRight.set(-launcherSpeedSlow.get());
    }

    public void intake(){
        launcherMotorLeft.set(-launcherIntakeSpeed.get());
        launcherMotorRight.set(launcherIntakeSpeed.get());

        feederMotorLeft.set(TalonSRXControlMode.PercentOutput, feederIntakeSpeed.get());
        feederMotorRight.set(TalonSRXControlMode.PercentOutput, -feederIntakeSpeed.get());
    }

    public void feed(){
        feederMotorLeft.set(TalonSRXControlMode.PercentOutput, -feederSpeed.get());
        feederMotorRight.set(TalonSRXControlMode.PercentOutput, feederSpeed.get());
    }

    public void stopFeed(){
        feederMotorLeft.set(TalonSRXControlMode.PercentOutput, 0.0);
        feederMotorRight.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    public void stopLauncher(){
        launcherMotorLeft.set(0.0);
        launcherMotorRight.set(0.0);
    }
}
