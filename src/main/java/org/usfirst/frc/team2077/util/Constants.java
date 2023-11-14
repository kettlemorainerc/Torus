package org.usfirst.frc.team2077.util;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class Frame{
        public static final double kWheelBaseLength = Units.inchesToMeters(19.25);
        public static final double kWheelBaseWidth = Units.inchesToMeters(22.5);
    }

    public static final class Drive {
        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kGuidingMotorCurrentLimit = 20; // amps

        public static final double kWheelRadius = Units.inchesToMeters(2);
        public static final double kWheelDiameter = kWheelRadius * 2.0;
        public static final double kWheelCircumference = kWheelDiameter * Math.PI;

        public static final double kDriveGearReduction = (45d * 22d) / (15d * 13d/*This is the variable gear*/);

    }
}


