package org.usfirst.frc.team2077.drivetrain;

import org.usfirst.frc.team2077.common.drivetrain.DriveModuleIF;

public interface SwerveModuleIF extends DriveModuleIF {
    void setAngle(double angle);

    double getAngle();
}
