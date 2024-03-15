package org.usfirst.frc.team2077;

import edu.wpi.first.cameraserver.CameraServer;
import org.usfirst.frc.team2077.common.HardwareRequirements;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.*;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;

public class RobotHardware extends HardwareRequirements<SwerveModule, SwerveChassis> {
    private static RobotHardware instance = null;

    public static RobotHardware getInstance() {
        if(instance == null) instance = new RobotHardware();
        return instance;
    }

    public final Climbers climbers;
    public final Intake intake;
    public final Launcher launcher;
    public final LauncherPivot pivot;
    public final Flippah flippah;

    private final SwerveChassis chassis;

    public RobotHardware() {
        instance = this;

        climbers = new Climbers();
        pivot    = new LauncherPivot();
        launcher = new Launcher();
        intake   = new Intake();
        chassis  = new SwerveChassis();
        flippah  = new Flippah();
    }

    @Override public SwerveChassis getChassis() {
        return chassis;
    }

    @Override public SwerveModule getWheel(WheelPosition position) {
        return chassis.getWheel(position);
    }
}
