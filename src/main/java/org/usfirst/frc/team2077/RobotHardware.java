package org.usfirst.frc.team2077;

import org.usfirst.frc.team2077.common.HardwareRequirements;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.Climbers;
import org.usfirst.frc.team2077.subsystem.Intake;
import org.usfirst.frc.team2077.subsystem.Launcher;
import org.usfirst.frc.team2077.subsystem.LauncherRotater;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;

public class RobotHardware extends HardwareRequirements<SwerveModule, SwerveChassis> {
    private static RobotHardware instance;

    public static RobotHardware getInstance() {
        if(instance == null) instance = new RobotHardware();
        return instance;
    }

    public final Climbers climbers = new Climbers();
    public final Intake intake = new Intake();
    public final LauncherRotater launcherRotater = new LauncherRotater();
    public final Launcher launcher = new Launcher();

    private final SwerveChassis chassis;

    public RobotHardware() {
        instance = this;

        chassis = new SwerveChassis();
    }

    @Override public SwerveChassis getChassis() {
        return chassis;
    }

    @Override public SwerveModule getWheel(WheelPosition position) {
        return chassis.getWheel(position);
    }
}
