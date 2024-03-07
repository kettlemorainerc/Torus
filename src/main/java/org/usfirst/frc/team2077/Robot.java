package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import org.usfirst.frc.team2077.command.autonomous.AutoMoveVelocityBased;
import org.usfirst.frc.team2077.command.autonomous.AutoRotate;

public class Robot extends TimedRobot {
    private RobotHardware hardware;
    private DriveStation driveStation;
    private SequentialCommandGroup commands;

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
//        Command auto = new AutoMoveVelocityBased(5,0);
//        auto.schedule();
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
    }

    /**
     * Called roughly every 1/50th second while the robot is "enabled" in "Teleoperated" mode
     */
    @Override public void teleopPeriodic() {}

    @Override public void teleopExit() {}

    @Override public void disabledInit() {}

    @Override public void disabledPeriodic() {}


}