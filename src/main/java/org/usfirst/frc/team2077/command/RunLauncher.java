package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Launcher;

public class RunLauncher extends RepeatedCommand {

    private final Launcher launcher;
    private final Launcher.Target target;

    public RunLauncher(Launcher.Target target){
        launcher = RobotHardware.getInstance().launcher;
        this.target = target;
    }

    @Override
    public void execute() {
        launcher.run(target);
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopLauncher();
    }
}
