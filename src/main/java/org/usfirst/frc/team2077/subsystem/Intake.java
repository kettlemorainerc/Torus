package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class Intake implements Subsystem {
    private TalonSRX motor;

    private SmartDashNumber speed = new SmartDashNumber("intake percent", 0.0, true);

    public Intake() {
        motor = new TalonSRX(13);
    }

    public void run(){
        motor.set(TalonSRXControlMode.PercentOutput, speed.get());
    }

    public void reverse(){
        motor.set(TalonSRXControlMode.PercentOutput, -speed.get());
    }

    public void stop(){
        motor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

}
