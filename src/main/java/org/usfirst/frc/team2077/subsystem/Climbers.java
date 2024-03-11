package org.usfirst.frc.team2077.subsystem;


import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class Climbers implements Subsystem {

    public enum RobotSide{
        LEFT, RIGHT
    }

    private final CANSparkMax rightMotor;
    private final CANSparkMax leftMotor;

    private final SmartDashRobotPreference speed = new SmartDashRobotPreference("climber percent", 0.8);

    public Climbers(){
        rightMotor = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushed);
        leftMotor = new CANSparkMax(10, CANSparkLowLevel.MotorType.kBrushed);

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
