package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;

public class RunIntake extends RepeatedCommand {
    /*
    if 0 degrees/radians is straight up, then angled at either
    -120 or 120 degrees/ -2/3 or 2/3 radians is the desired
    angle for intake

    still need to make launcher angle subsystem to get the angle
     */
    private double launcherAngle;
    private double angleDeadZone = 0.05;
    private double currentAngleSign;


    public RunIntake(){

    }

    public void initialize(){
        currentAngleSign = Math.signum(launcherAngle) - angleDeadZone;

    }

    @Override
    public void execute() {
        if(currentAngleSign >= 0.95){
            RobotHardware.getInstance().intake.FrontIntakeRun();
        } else if (currentAngleSign <= -0.95) {
            RobotHardware.getInstance().intake.BackIntakeRun();
        } else if (-0.95 < currentAngleSign && currentAngleSign < 0.95) {
            RobotHardware.getInstance().intake.StopIntake();
        }

    }

    @Override
    public void end(boolean interrupted) {

    }
}
