package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;
import org.usfirst.frc.team2077.util.SmartDashNumber;
import org.usfirst.frc.team2077.util.SmartDashRobotPreference;

public class SetLauncherAngle extends SelfDefinedCommand {

    private SmartDashRobotPreference setAngle;
    private LauncherPivot pivot;

    public SetLauncherAngle(String key, double defaultValue){
        pivot = RobotHardware.getInstance().pivot;
        setAngle = new SmartDashRobotPreference(key, defaultValue);
    }

    @Override
    public void initialize() {
        pivot.setTarget(setAngle.get());
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
