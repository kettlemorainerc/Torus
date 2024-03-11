package org.usfirst.frc.team2077.command.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;

import java.awt.*;
import java.util.ArrayList;

public class StraightenWheels extends Command {

    private final SwerveChassis chassis;
    private final ArrayList<SwerveModule> modules;

    private double targetAngle;

    public StraightenWheels(double targetAngle){
        this.targetAngle = targetAngle;
        chassis = RobotHardware.getInstance().getChassis();
        modules = new ArrayList<>(chassis.getDriveModules().values());
    }

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
        modules.stream().map(SwerveModule::getGuidingMotor).forEach(e -> e.setAngleForced(targetAngle));
    }

    @Override
    public void end(boolean interrupted){
        chassis.halt();
    }

    @Override
    public boolean isFinished(){
        return modules.stream().allMatch(SwerveModule::isAtAngle);
    }
}
