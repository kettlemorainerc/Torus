package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.command.autonomous.*;
import org.usfirst.frc.team2077.subsystem.Launcher;
import org.usfirst.frc.team2077.util.SmartDashNumber;

public class Robot extends TimedRobot {
    private RobotHardware hardware;
    private DriveStation driveStation;
    private int autoTick;
    private SmartDashNumber autoDash = new SmartDashNumber("autonomous number: ", 0.0, true);
    @Override public void robotInit() {
        hardware = new RobotHardware();
        driveStation = new DriveStation();

    }

    @Override public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    /**
     * When you click the "Autonomous" option in driver station
     */

    @Override public void autonomousInit() {
        autoTick = 0;
    }

    /**
     * When you click the "Teleoperated" option in driver station
     */
    @Override public void teleopInit() {
    }

    /**
     * Called roughly every 1/50th second while the robot is "enabled" in "Autonomous" mode
     */

    @Override public void autonomousPeriodic() {
        autoTick++;
        if(autoTick == 1){
            SequentialCommandGroup auto = new SequentialCommandGroup();
            int autonomousNumber = autoDash.get().intValue();

            double d, a; //Java is very funky, and aparently I can't redeclar a variable in a seperate case because it is the same scope.
            switch(autonomousNumber) {
                case 0:
                    //Moves robot straight forward
                    auto.addCommands(
                        new AutoSwerveMoveOdometryBased(3, 0)
                    );
                    break;
                case 1:
                    //Shoots in Speaker when aligned with the left face of the Speaker, then drives towards the center of the field
                    d = 5;
                    a = -2.443;
                    auto.addCommands(
                        new AutoLaunch(Launcher.Target.SPEAKER),
                        new AutoSwerveMoveOdometryBased(
                            Math.cos(a) * d, Math.sin(a) * d
                        )
                    );
                    break;
                case 2:
                    //Shoots in Speaker when aligned with the middle face of the Speaker, then drives backwards
                    auto.addCommands(
                        new AutoLaunch(Launcher.Target.SPEAKER),
                        new AutoSwerveMoveOdometryBased(-3, 0)
                    );
                    break;
                case 3:
                    //Shoots in Speaker when aligned with the right face of the Speaker, then drives towards the center of the field.
                    d = 5;
                    a = 2.443;
                    auto.addCommands(
                        new AutoLaunch(Launcher.Target.SPEAKER),
                        new AutoSwerveMoveOdometryBased(
                            Math.cos(a) * d, Math.sin(a) * d
                        )
                    );
                    break;
                case 4:
                    //Shoots in Amp and then drives sideways
                    auto.addCommands(
                        new AutoLaunch(Launcher.Target.AMP),
                        new AutoSwerveMoveOdometryBased(0, 3)
                    );
                    break;
            }

            auto.schedule();
        }

    }

    /**
     * Called roughly every 1/50th second while the robot is "enabled" in "Teleoperated" mode
     */
    @Override public void teleopPeriodic() {}

    @Override public void teleopExit() {}

    @Override public void disabledInit() {}

    @Override public void disabledPeriodic() {}


}