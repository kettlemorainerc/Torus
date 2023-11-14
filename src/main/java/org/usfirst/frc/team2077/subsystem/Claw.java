package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.common.subsystem.InputMap;

public class Claw implements Subsystem {

    public enum Input{
        CLOSE;
    }

    private final static double maxSpeed = 0.8;

    private TalonSRX motor = new TalonSRX(11);

    public Claw(){
        this.register();
    }

    @Override public void periodic() {
//        double openPercent = InputMap.getInput("Open");
        double closePercent = InputMap.getInput(Input.CLOSE);

        double percent = closePercent;

        setMotor(percent);
    }

    private void setMotor(double percent){
        percent *= maxSpeed;
        motor.set(TalonSRXControlMode.PercentOutput, percent);
    }

}
