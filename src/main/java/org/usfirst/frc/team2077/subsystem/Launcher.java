package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class Launcher implements Subsystem {
    private final CANSparkMax launcherMotorLeft, launcherMotorRight;

    private final CANSparkMax feederMotorLeft, feederMotorRight;

    private final SmartDashRobotPreference launcherSpeedFast = new SmartDashRobotPreference("fast launcher launch percent", 0.525);
    private final SmartDashRobotPreference launcherSpeedSlow = new SmartDashRobotPreference("slow launcher launch percent", 0.02);
    private final SmartDashRobotPreference feederSpeed = new SmartDashRobotPreference("feeder feed percent", 1.0);

    private final SmartDashRobotPreference launcherIntakeSpeed = new SmartDashRobotPreference("launcher intake percent", 0.325);
    private final SmartDashRobotPreference feederIntakeSpeed = new SmartDashRobotPreference("feeder intake percent", 0.2);

    private final PowerDistribution PDH;

    private final SmartDashNumber outputVoltageChange = new SmartDashNumber("feeder voltage delta", 0.0, true);

    private double[] voltageSamples = new double[10];

    public Launcher(){
        launcherMotorLeft = new CANSparkMax(11, CANSparkMaxLowLevel.MotorType.kBrushless);
        launcherMotorRight = new CANSparkMax(12, CANSparkMaxLowLevel.MotorType.kBrushless);

        launcherMotorLeft.setIdleMode(CANSparkMax.IdleMode.kCoast);
        launcherMotorRight.setIdleMode(CANSparkMax.IdleMode.kCoast);

        feederMotorLeft = new CANSparkMax(13, CANSparkMaxLowLevel.MotorType.kBrushed);
        feederMotorRight = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushed);

        feederMotorLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
        feederMotorRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

        PDH = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);
    }

    public void runLauncherFast(){
        launcherMotorLeft.set(-launcherSpeedFast.get());
        launcherMotorRight.set(launcherSpeedFast.get());
    }

    public void runLauncherSlow(){
        launcherMotorLeft.set(-launcherSpeedSlow.get());
        launcherMotorRight.set(launcherSpeedSlow.get());
    }

    public void intake() {
        launcherMotorLeft.set(launcherIntakeSpeed.get());
        launcherMotorRight.set(-launcherIntakeSpeed.get());

        feederMotorLeft.set(-feederIntakeSpeed.get());
        feederMotorRight.set(feederIntakeSpeed.get());

        //TODO: Determine if this is a reliable way of detecting a ring in the Launcher, see if this can be used in other applications
//        double voltage = feederMotorLeft.getAppliedOutput();
//        double loss = PDH.getVoltage() * feederMotorLeft.get() - voltage;
    }

    public void feed(){
        feederMotorLeft.set(feederSpeed.get());
        feederMotorRight.set(-feederSpeed.get());
    }

    public void stopFeed(){
        feederMotorLeft.set(0.0);
        feederMotorRight.set(0.0);
    }

    public void stopLauncher(){
        launcherMotorLeft.set(0.0);
        launcherMotorRight.set(0.0);
    }
}
