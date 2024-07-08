package org.usfirst.frc.team2077.command;




import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.SelfDefinedCommand;
import org.usfirst.frc.team2077.subsystem.Launcher;
import org.usfirst.frc.team2077.common.Clock;



public class QuickTapShoot extends SelfDefinedCommand {

    private final Launcher launcher;
    private final Launcher.Target target;
    private double startTime;


    public QuickTapShoot(Launcher.Target target){
        launcher = RobotHardware.getInstance().launcher;
        this.target = target;
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
        startTime = Clock.getSeconds();
    }

    @Override
    public void execute() {
        launcher.run(target);
        if(startTime - Clock.getSeconds() >= 50){
            launcher.stopFeed();
        }
    }

    @Override
    public void end(boolean interrupted) {

    }
}
