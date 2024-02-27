package org.usfirst.frc.team2077.command;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class RotateLauncher extends RepeatedCommand {

    private int d;
    private LauncherPivot pivot;

    private SmartDashNumber speed = new SmartDashNumber("Launcher rotator speed", 0.0, true);

    public RotateLauncher(int d){
        pivot = RobotHardware.getInstance().pivot;
        this.d = d;
    }

    @Override
    public void execute() {
        pivot.run(d * speed.get());
        pivot.setTargeting(false);
    }


    @Override
    public void end(boolean interrupted) {
        pivot.stop();
    }
}
