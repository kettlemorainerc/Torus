package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;

public class CalibratePivot extends SelfDefinedCommand {

    private LauncherPivot pivot;

    public CalibratePivot(){
        pivot = RobotHardware.getInstance().pivot;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        pivot.reset();
    }

    @Override
    public void execute() {
        ;
    }

    @Override
    public void end(boolean interrupted) {
        ;
    }
}
