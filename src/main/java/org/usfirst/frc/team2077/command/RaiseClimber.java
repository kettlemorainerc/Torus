package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Climbers;

public class RaiseClimber extends RepeatedCommand {

    public enum Direction{
        RAISE, LOWER;
    }

    private final Climbers.RobotSide side;
    private final Direction direction;

    private final Climbers climbers;

    public RaiseClimber(Climbers.RobotSide side, Direction direction){
        climbers = RobotHardware.getInstance().climbers;

        this.side = side;
        this.direction = direction;
    }

    public void initialize(){
        switch(direction){
            case RAISE:
                climbers.raise(side);
                break;
            case LOWER:
                climbers.lower(side);
                break;
        }
    }

    @Override
    public void execute() {
        ;
    }

    @Override
    public void end(boolean interrupted) {
        climbers.stop(side);
    }

}
