package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;

public class ResetGyro extends SelfDefinedCommand {

    boolean finished = false;

    public ResetGyro(){
        ;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        RobotHardware.getInstance().getChassis().resetGyro();
        finished = true;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
