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
//        // keep this first in the scheduler:
//        Command straightenWheels = new StraightenWheels(0);
//        Command launch = new AutoLaunch(Launcher.Target.SPEAKER);
////        Command move = new AutoMoveEncoderBased()
//        Command move = new AutoSwerveMoveOdometryBased(1.0, 0.0);
//        //Command forward = new AutoMoveEncoderBased(10,0);
//
//        // add commands here (keep straightenWheels first):
//        SequentialCommandGroup autonomous = new SequentialCommandGroup(straightenWheels, launch, move);
//        autonomous.schedule();
        //forward.schedule();



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
        autoTick ++;
        if(autoTick ==1){
            int autonomousNumber = autoDash.get().intValue();
            Command launchSpeaker = new AutoLaunch(Launcher.Target.SPEAKER);

            switch (autonomousNumber) {
                case(0):
                    // Straight Forward
                    SequentialCommandGroup straightForwardCommands = new SequentialCommandGroup(new StraightenWheels(0), new AutoSwerveMoveOdometryBased(10,0));
                    straightForwardCommands.schedule();
                    break;
                case(1):
                    //Shoot in speaker (line up on left angle side)
                    Command straightenWheelsLeft = new StraightenWheels(-2.443);
                    Command backwardLeft = new AutoSwerveMoveOdometryBased(Math.pow(Math.cos(-2.443),2)*10,Math.pow(Math.sin(-2.443),2)*10);
                    SequentialCommandGroup leftSpeakerCommands = new SequentialCommandGroup(straightenWheelsLeft,launchSpeaker, backwardLeft);
                    leftSpeakerCommands.schedule();
                    break;

                case(2):
                    //Shoot in speaker (line up in middle)
                    Command straightenWheelsBackwards = new StraightenWheels(Math.PI);
                    Command backwards = new AutoSwerveMoveOdometryBased(-10,0);
                    SequentialCommandGroup middleSpeakerCommands = new SequentialCommandGroup(straightenWheelsBackwards, launchSpeaker, backwards);
                    middleSpeakerCommands.schedule();
                    break;
                case(3):
                    //Shoot in speaker (line up on right)
                    Command straightenWheelsRight = new StraightenWheels(2.443);
                    Command backwardRight = new AutoSwerveMoveOdometryBased(Math.pow(Math.cos(2.443),2)*10,Math.pow(Math.sin(2.443),2)*10);
                    SequentialCommandGroup rightSpeakerCommands = new SequentialCommandGroup(straightenWheelsRight, launchSpeaker, backwardRight);
                    rightSpeakerCommands.schedule();
                    break;
            }
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