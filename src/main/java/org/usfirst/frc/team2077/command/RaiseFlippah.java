package org.usfirst.frc.team2077.command;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class RaiseFlippah extends RepeatedCommand {

    private final SmartDashRobotPreference speed = new SmartDashRobotPreference("Flippah speed", 0.6);
    private final CANSparkMax motor;

    public RaiseFlippah(){
        motor = new CANSparkMax(17, CANSparkLowLevel.MotorType.kBrushed);
        motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
    }

    @Override
    public void initialize(){
        motor.set(-speed.get());
    }

    @Override
    public void execute() {
        ;
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0.0);
    }
}
