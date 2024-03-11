package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.subsystem.Intake;
import org.usfirst.frc.team2077.subsystem.Launcher;

public class RunIntake extends RepeatedCommand {

    private final Direction direction;
    private final Intake intake;
    private final Launcher launcher;

    public enum Direction{
        IN,
        OUT
    }

    public RunIntake(Direction d){
        intake = RobotHardware.getInstance().intake;
        launcher = RobotHardware.getInstance().launcher;
        direction = d;
    }

    public void initialize(){
    }

    @Override
    public void execute() {
        switch(direction){
            case IN:
                intake.run();
                launcher.run(Launcher.Target.INTAKE);
                break;
            case OUT:
                intake.reverse();
                break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        launcher.stopLauncher();
        launcher.stopFeed();
    }
}
