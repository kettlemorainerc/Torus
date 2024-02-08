package org.usfirst.frc.team2077;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.usfirst.frc.team2077.common.*;
import org.usfirst.frc.team2077.drivetrain.SwerveChassis;
import org.usfirst.frc.team2077.subsystem.*;
import org.usfirst.frc.team2077.subsystem.swerve.SwerveModule;

public class RobotHardware extends HardwareRequirements<SwerveChassis, SwerveModule, RectangularWheelPosition> {
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
        super(new SubsystemBase() {}, new SubsystemBase() {}, null, null);
        instance = this;

        chassis = new SwerveChassis();
    }

    @Override public SwerveChassis getChassis() {
        return chassis;
    }

    @Override public SwerveModule getWheel(RectangularWheelPosition position) {
        return chassis.getWheel(position);
    }
}
