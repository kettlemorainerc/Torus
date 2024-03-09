package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.Launcher;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class SetLauncherAngle extends SelfDefinedCommand {

    private Launcher.Target target;
    private LauncherPivot pivot;

    public SetLauncherAngle(Launcher.Target target){
        pivot = RobotHardware.getInstance().pivot;
        this.target = target;
    }

    @Override
    public void initialize() {
        pivot.setTarget(target);
    }

    @Override
    public void execute() {
        ;
    }

    @Override
    public boolean isFinished() {
        return pivot.atTarget();
    }

    @Override
    public void end(boolean interrupted) {
        ;
    }
}
