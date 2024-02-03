package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Intake implements Subsystem {
    //Banebots RS550 motors
    //TODO: device number tbd
    private TalonSRX FrontMotor = new TalonSRX(0);
    private TalonSRX BackMotor = new TalonSRX(0);

    private double speed = 1;
    //TODO: speed tbd

    public Intake(){

    }

    public void FrontIntakeRun(){
        FrontMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
    public void BackIntakeRun(){
        BackMotor.set(TalonSRXControlMode.PercentOutput, speed);
    }
    public void StopIntake(){
        BackMotor.set(TalonSRXControlMode.PercentOutput, 0f);

    }

}
