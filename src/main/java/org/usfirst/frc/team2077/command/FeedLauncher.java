package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Launcher;

public class FeedLauncher extends RepeatedCommand {

    private final Launcher launcher;

    public FeedLauncher(){
        launcher = RobotHardware.getInstance().launcher;
    }

    @Override
    public void execute() {
        launcher.feed();
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopFeed();
    }
}
