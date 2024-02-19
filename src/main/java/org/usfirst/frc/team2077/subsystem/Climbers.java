package org.usfirst.frc.team2077.subsystem;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class Climbers implements Subsystem {
    private TalonSRX rightMotor = new TalonSRX(14);
    private TalonSRX leftMotor = new TalonSRX(15);

    private SmartDashNumber speed = new SmartDashNumber("climber percent", 0.0, true);

    public Climbers(){

    }

    public void raise(boolean left, boolean right){
        if(left) leftMotor.set(TalonSRXControlMode.PercentOutput, -speed.get());
        if(right) rightMotor.set(TalonSRXControlMode.PercentOutput, speed.get());
    }

    public void lower(boolean left, boolean right){
        if(left) leftMotor.set(TalonSRXControlMode.PercentOutput, speed.get());
        if(right) rightMotor.set(TalonSRXControlMode.PercentOutput, -speed.get());
    }

    public void stop(){
        leftMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
        rightMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

}
