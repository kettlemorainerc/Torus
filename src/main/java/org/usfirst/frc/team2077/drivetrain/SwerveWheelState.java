package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;

public interface SwerveWheelState extends DriveModuleIF {
    void setAngle(double angle);

    double getAngle();
}
