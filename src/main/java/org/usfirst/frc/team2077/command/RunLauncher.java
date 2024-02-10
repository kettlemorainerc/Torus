package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Launcher;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class RunLauncher extends RepeatedCommand {


    private int speed = 0;
    private int dir = 0;

    public RunLauncher(int speed, int dir){
        this.speed = speed;
        this.dir = dir;
    }


    @Override
    public void execute() {
        RobotHardware.getInstance().launcher.Launch(dir);
    }

    @Override
    public void end(boolean interrupted) {
        RobotHardware.getInstance().launcher.Stop();
    }
}
