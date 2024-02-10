package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Launcher implements Subsystem {
    private TalonSRX launchingMotor = new TalonSRX(0);

    public Launcher(){

    }
    public void Launch(double dir){
        launchingMotor.set(TalonSRXControlMode.PercentOutput, dir);
    }
    public void Stop(){
        launchingMotor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
