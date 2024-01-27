package org.usfirst.frc.team2077.subsystem;


import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climbers implements Subsystem {
    //TODO: determine device numbers
    private TalonSRX rightMotor = new TalonSRX(0);
    private TalonSRX leftMotor = new TalonSRX(0);

    //TODO: determine directions
    private double leftUpDirection = 1;
    private double rightUpDirection = -1;
    //TODO: speed TBD
    private double speed = 0.1d;

    public enum Direction {
        UP,
        DOWN;
    }

    public Climbers(){
        //Dear Henry
        //Yes
        //Sincerely, Dustin
    }


    public void raise(){
        leftMotor.set(TalonSRXControlMode.PercentOutput, leftUpDirection * speed);
        rightMotor.set(TalonSRXControlMode.PercentOutput, rightUpDirection * speed);
    }
    public void lower(){
        leftMotor.set(TalonSRXControlMode.PercentOutput, -leftUpDirection * speed);
        rightMotor.set(TalonSRXControlMode.PercentOutput, -rightUpDirection * speed);
    }

}
