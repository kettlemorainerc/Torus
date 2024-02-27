package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Launcher;

public class RunLauncher extends RepeatedCommand {

    public enum Speed{
        FAST, SLOW;
    }

    private Launcher launcher;
    private Speed speed;

    public RunLauncher(Speed speed){
        launcher = RobotHardware.getInstance().launcher;
        this.speed = speed;
    }

    @Override
    public void execute() {
        switch(speed){
            case FAST:
                launcher.runLauncherFast();
                break;
            case SLOW:
                launcher.runLauncherSlow();
                break;
            }

    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopLauncher();
    }
}
