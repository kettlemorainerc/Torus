package org.usfirst.frc.team2077.subsystem;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class Flippah implements Subsystem {

    public enum Position{
        UP,
        DOWN,
        UNKNOWN
    }

    private final SmartDashRobotPreference speed = new SmartDashRobotPreference("Flippah Speed", 0.1);
    private final CANSparkMax motor;

    private final DigitalInput lower;
    private final DigitalInput upper;

    private Position p = Position.UNKNOWN;
    private Position t = Position.UNKNOWN;

    public Flippah(){
        motor = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushed);
        motor.setIdleMode(CANSparkBase.IdleMode.kBrake);

        lower = new DigitalInput(3);
        upper = new DigitalInput(4);

        this.register();
    }

    @Override
    public void periodic(){
        switch(t){
            case UP:
                up();
                break;
            case DOWN:
                down();
                break;
            default:
                motor.set(0.0);
        }
    }

    public void setTarget(Position t){
        this.t = t;
    }

    private void up(){
        if(upper.get()){
            motor.set(0.0);
            return;
        }

        motor.set(speed.get());
    }

    private void down(){
        if(lower.get()){
            motor.set(0.0);
            return;
        }

        motor.set(-speed.get());
    }


}
