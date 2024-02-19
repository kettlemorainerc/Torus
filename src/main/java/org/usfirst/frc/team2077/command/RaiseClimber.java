package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Climbers;

public class RaiseClimber extends RepeatedCommand {

    public enum Climber{
        LEFT, RIGHT;
    }

    public enum Direction{
        RAISE, LOWER;
    }

    private Climber climber;
    private Direction direction;

    private Climbers climbers;

    public RaiseClimber(Climber climber, Direction direction){
        climbers = RobotHardware.getInstance().climbers;

        this.climber = climber;
        this.direction = direction;
    }

    public void initialize(){
    }

    @Override
    public void execute() {
        switch(direction){
            case RAISE:
                climbers.raise(climber == Climber.LEFT, climber == Climber.RIGHT);
                break;
            case LOWER:
                climbers.lower(climber == Climber.LEFT, climber == Climber.RIGHT);
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        climbers.stop();
    }

}
