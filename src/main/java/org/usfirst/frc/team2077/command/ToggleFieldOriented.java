package org.usfirst.frc.team2077.command;

import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.command.RepeatedCommand;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;

public class ToggleFieldOriented extends RepeatedCommand {

    private SwerveChassis chassis;

    public ToggleFieldOriented(){
        chassis = RobotHardware.getInstance().getChassis();
    }

    public void initialize(){
        chassis.setFieldOriented(false);
    }

    @Override
    public void execute() {
        ;
    }

    @Override
    public void end(boolean interrupted) {
        chassis.setFieldOriented(true);
    }
}
