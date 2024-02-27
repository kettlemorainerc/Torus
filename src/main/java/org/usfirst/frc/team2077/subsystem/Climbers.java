package org.usfirst.frc.team2077.subsystem;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class Climbers implements Subsystem {

    public enum RobotSide{
        LEFT, RIGHT;
    }

    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;

    private SmartDashRobotPreference speed = new SmartDashRobotPreference("climber percent", 0.8);

    public Climbers(){
        rightMotor = new CANSparkMax(9, CANSparkMaxLowLevel.MotorType.kBrushed);
        leftMotor = new CANSparkMax(10, CANSparkMaxLowLevel.MotorType.kBrushed);

        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void raise(RobotSide side){
        if(side == RobotSide.LEFT)       leftMotor.set(-speed.get());
        else if(side == RobotSide.RIGHT) rightMotor.set(speed.get());
    }

    public void lower(RobotSide side){
        if(side == RobotSide.LEFT)       leftMotor.set(speed.get());
        else if(side == RobotSide.RIGHT) rightMotor.set(-speed.get());
    }

    public void stop(RobotSide side){
        if(side == RobotSide.LEFT)       leftMotor.set(0.0);
        else if(side == RobotSide.RIGHT) rightMotor.set(0.0);
    }

}
