package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.LauncherRotater;

public class RotateLauncher extends RepeatedCommand {
    LauncherRotater.InputDir moveDir;

    public RotateLauncher(LauncherRotater.InputDir direction){
         moveDir = direction;
    }


    @Override
    public void execute() {
        if(moveDir == LauncherRotater.InputDir.FORWARD){
            RobotHardware.getInstance().launcherRotater.RotateForward();
        }else if(moveDir == LauncherRotater.InputDir.BACKWARD){
            RobotHardware.getInstance().launcherRotater.RotateBackward();
        }else if(moveDir == LauncherRotater.InputDir.FRONT) {
            RobotHardware.getInstance().launcherRotater.RotateToIntake();
        }
    }


    @Override
    public void end(boolean interrupted) {

    }
}
