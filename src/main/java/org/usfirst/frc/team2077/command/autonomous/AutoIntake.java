package org.usfirst.frc.team2077.command.autonomous;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.math.Position;
import org.usfirst.frc.team2077.subsystem.Intake;
import org.usfirst.frc.team2077.subsystem.Launcher;
import org.usfirst.frc.team2077.subsystem.LauncherPivot;

import static org.usfirst.frc.team2077.common.VelocityDirection.FORWARD;

public class AutoIntake extends AutoSwerveMoveOdometryBased {

    private static final double maxIntakeDistance = 1.0; //Meters

    private final Launcher launcher;
    private final LauncherPivot pivot;
    private final Intake intake;

    //Autonomous command that drives forward and intakes simultaneously
    public AutoIntake(double forward){
        super(forward, 0.0, 0.0);

        if(forward < 0.0){
            System.out.println("I wouldn't recommend driving backwards into a ring");
            System.out.println("but sure, you do you");
        }

        RobotHardware rh = RobotHardware.getInstance();
        intake   = rh.intake;
        launcher = rh.launcher;
        pivot    = rh.pivot;
    }

    @Override
    public void initialize() {
        super.initialize();
        //The pivot may not reach the intake position before the robot has made it
        //to the ring. I have employed the Ostrich Algorithm to solve this.
        pivot.setTarget(Launcher.Target.INTAKE);
    }

    @Override
    public void execute(){
        super.execute();

        Position current = chassis.getPosition();

        double deltaForward = target.get(FORWARD) - current.get(FORWARD);

        if(Math.abs(deltaForward) < maxIntakeDistance && pivot.atTarget() && !intake.running()){
            intake.run();
            launcher.run(Launcher.Target.INTAKE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        launcher.stopLauncher();
        launcher.stopFeed();
        super.end(interrupted);
    }
}
