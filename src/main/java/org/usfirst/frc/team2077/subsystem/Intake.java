package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class Intake implements Subsystem {
    private TalonSRX motor;

    private SmartDashRobotPreference speed = new SmartDashRobotPreference("intake percent", 0.6);

    public Intake() {
        motor = new TalonSRX(16);

        motor.setNeutralMode(NeutralMode.Coast);
    }

    public void run(){
        motor.set(ControlMode.PercentOutput, speed.get());
    }

    public void reverse(){
        motor.set(TalonSRXControlMode.PercentOutput, -speed.get());
    }

    public void stop(){
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

}
