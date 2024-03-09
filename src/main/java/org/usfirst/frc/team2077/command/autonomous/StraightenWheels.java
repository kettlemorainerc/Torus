package org.usfirst.frc.team2077.command.autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import org.usfirst.frc.team2077.RobotHardware;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.drivetrain.AbstractChassis;

import java.awt.*;

public class StraightenWheels extends Command {
    private final double DEADBAND = 0.196;
    private AbstractChassis chassis;
    private RobotHardware hardware;
    double targetAngle;
    public StraightenWheels(double targetAngle){
        this.targetAngle = targetAngle;
    }

    @Override
    public void initialize(){
        hardware = RobotHardware.getInstance();
        chassis = hardware.getChassis();
    }

    @Override
    public void execute(){
        hardware.getWheel(WheelPosition.FRONT_LEFT).getGuidingMotor().setAngleForced(0);
        hardware.getWheel(WheelPosition.FRONT_RIGHT).getGuidingMotor().setAngleForced(0);
        hardware.getWheel(WheelPosition.BACK_RIGHT).getGuidingMotor().setAngleForced(0);
        hardware.getWheel(WheelPosition.BACK_LEFT).getGuidingMotor().setAngleForced(0);
        if(this.isFinished()){
            this.end(false);
        }
    }

    @Override
    public void end(boolean interrupted){
        chassis.halt();
    }

    @Override
    public boolean isFinished(){
        return
        Math.abs(hardware.getWheel(WheelPosition.BACK_LEFT).getGuidingMotor().getAngle() - targetAngle) < DEADBAND &&
                Math.abs(hardware.getWheel(WheelPosition.BACK_RIGHT).getGuidingMotor().getAngle() - targetAngle) < DEADBAND &&
                Math.abs(hardware.getWheel(WheelPosition.FRONT_RIGHT).getGuidingMotor().getAngle() - targetAngle) < DEADBAND &&
                Math.abs(hardware.getWheel(WheelPosition.FRONT_LEFT).getGuidingMotor().getAngle() - targetAngle) < DEADBAND;

    }
}
