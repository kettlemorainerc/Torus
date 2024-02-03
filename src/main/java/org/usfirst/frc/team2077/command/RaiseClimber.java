package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Climbers;



public class RaiseClimber extends RepeatedCommand {

    private Climbers.Direction moveDir;

    public RaiseClimber(Climbers.Direction direction){
       moveDir = direction;
    }

    public void initialize(){

    }

    @Override
    public void execute() {
        if(moveDir == Climbers.Direction.UP) {
            RobotHardware.getInstance().climbers.raise();
        }else{
            RobotHardware.getInstance().climbers.lower();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
