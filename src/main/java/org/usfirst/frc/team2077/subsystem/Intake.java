package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class Intake implements Subsystem {

    private final SmartDashRobotPreference speed = new SmartDashRobotPreference("intake percent", 0.6);
    private final TalonSRX motor;
    private final LauncherPivot pivot;
    
    public Intake() {
        motor = new TalonSRX(16);

        motor.setNeutralMode(NeutralMode.Coast);
        pivot = RobotHardware.getInstance().pivot;
    }

    public void run(){
        if(!pivot.atTarget() || pivot.getTarget() != Launcher.Target.INTAKE){
            stop();
            return;
        }

        motor.set(ControlMode.PercentOutput, speed.get());
    }

    public void reverse(){
        motor.set(TalonSRXControlMode.PercentOutput, -speed.get());
    }

    public boolean running(){
        return motor.getMotorOutputPercent() != 0.0;
    }

    public void stop(){
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

}
