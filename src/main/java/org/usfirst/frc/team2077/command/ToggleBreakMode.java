package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;

public class ToggleBreakMode extends RepeatedCommand {

    private final SwerveChassis chassis;

    public ToggleBreakMode(){
        chassis = RobotHardware.getInstance().getChassis();
    }

    public void initialize(){
        chassis.mode = SwerveChassis.DriveMode.BRAKE;
    }

    @Override
    public void execute() {
        ;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.mode = SwerveChassis.DriveMode.COAST;
    }
}
