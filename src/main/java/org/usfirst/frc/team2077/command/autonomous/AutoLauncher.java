package org.usfirst.frc.team2077.command.autonomous;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
//TODO
public class AutoLauncher extends CommandBase {
    TalonSRX motor1;
    TalonSRX motor2;

    double shootTime;
    double startTime;
    RobotHardware hardware;
    public AutoLauncher(double time, RobotHardware hardware){
        shootTime = time;
        this.hardware = hardware;

    }

    @Override
    public void initialize(){
        startTime = Clock.getSeconds();
        motor1.set(TalonSRXControlMode.PercentOutput, 0.2725);
        motor2.set(TalonSRXControlMode.PercentOutput, 0.2725);
    }

    @Override
    public void execute(){
        if(shootTime <= Clock.getSeconds() - startTime){
            this.end(false);
        }
    }


    @Override
    public void end(boolean interrupted) {
        motor1.set(TalonSRXControlMode.PercentOutput, 0.0);
        motor2.set(TalonSRXControlMode.PercentOutput, 0.0);
    }

    @Override
    public boolean isFinished(){
        return(shootTime <= Clock.getSeconds() - startTime);
    }
}
