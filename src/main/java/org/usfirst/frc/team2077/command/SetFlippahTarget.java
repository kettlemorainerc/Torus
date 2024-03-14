package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.Robot;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.Flippah;

public class SetFlippahTarget extends SelfDefinedCommand {

    private final Flippah.Position target;
    private final Flippah flippah;

    public SetFlippahTarget(Flippah.Position target){
        flippah = RobotHardware.getInstance().flippah;
        this.target = target;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void initialize() {
        flippah.setTarget(target);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }
}
