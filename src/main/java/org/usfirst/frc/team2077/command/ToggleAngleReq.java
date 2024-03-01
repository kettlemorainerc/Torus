package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;

public class ToggleAngleReq extends RepeatedCommand {

    private SwerveChassis chassis;

    public ToggleAngleReq(){
        chassis = RobotHardware.getInstance().getChassis();
    }

    public void initialize(){
        chassis.mode = SwerveChassis.DriveMode.ANGLE_REQ;
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
