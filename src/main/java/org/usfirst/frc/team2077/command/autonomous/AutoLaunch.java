package org.usfirst.frc.team2077.command.autonomous;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.subsystem.Launcher;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;

public class AutoLaunch extends CommandBase {

    private enum State{
        TO_ANGLE,
        SPIN_UP,
        END;
    }

    private Launcher launcher;
    private LauncherPivot pivot;

    private Launcher.Target target;
    private State state = null;

    private int launchTimer;

    public AutoLaunch(Launcher.Target target){
        launcher = RobotHardware.getInstance().launcher;
        pivot = RobotHardware.getInstance().pivot;
        this.target = target;
    }

    @Override
    public void initialize(){
        updateState(State.TO_ANGLE);
        launchTimer = 50;
    }

    @Override
    public void execute(){
        switch(state){
            case TO_ANGLE:
                if(pivot.atTarget()) updateState(State.SPIN_UP);
                return;
            case SPIN_UP:
                if(launcher.atSpeed()) {
                    launchTimer--;
                    if (launchTimer < 0) updateState(State.END);
                }
                return;
        }
    }

    public void updateState(State newState){
        if(state == newState) return;

        state = newState;

        switch(state){
            case TO_ANGLE:
                pivot.setTarget(target);
                return;
            case SPIN_UP:
                launcher.run(target);
                launcher.feed();
                return;
            case END:
                end(false);
        }
    }


    @Override
    public void end(boolean interrupted) {
        pivot.stop();
        launcher.stopLauncher();
        launcher.stopFeed();
    }

    @Override
    public boolean isFinished(){
        return state == State.END;
    }
}
