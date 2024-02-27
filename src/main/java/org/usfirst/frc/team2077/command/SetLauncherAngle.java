package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class SetLauncherAngle extends RepeatedCommand {

    private double angle;
    private LauncherPivot pivot;

    private SmartDashNumber speed = new SmartDashNumber("Launcher rotator speed", 0.0, true);

    public SetLauncherAngle(double angle){
        pivot = RobotHardware.getInstance().pivot;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        pivot.setTarget(angle);
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
