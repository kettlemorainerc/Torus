package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;
import org.usfirst.frc.team2077.util.SmartDash.SmartDashNumber;

public class RotateLauncher extends RepeatedCommand {

    private LauncherPivot pivot;
    private int d;

    private SmartDashNumber speed = new SmartDashNumber("Launcher rotator speed", 0.0, true);

    //TODO: remove
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
