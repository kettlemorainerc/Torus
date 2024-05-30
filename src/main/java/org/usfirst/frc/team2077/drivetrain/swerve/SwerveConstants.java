package org.usfirst.frc.team2077.drivetrain.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveConstants {
    public static final double wheelBaseLength = Units.inchesToMeters(29.5);
    public static final double wheelBaseWidth = Units.inchesToMeters(29.5);

    public static final double wheelDiameter = Units.inchesToMeters(2.9);
    public static final double wheelRadius = 0.5 * wheelDiameter;
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double azimuthRatio = 203d / 9424d;
    public static final double driveGearReduction = (45d * 22d) / (15d * 13d/*This is the variable gear*/);

    public static final int guidingMotorCurrentLimit = 20; // amps
    public static final int drivingMotorCurrentLimit = 40; // amps

    public enum MotorPosition{
        FRONT_LEFT (2, 1, 1.5, /*P:*/ 0.02048513852059841, /*I:*/ 5.435076891444623E-4, 0.05),
        BACK_LEFT  (8, 7, 1,   /*P:*/ 0.030933115631341934, /*I:*/ 6.17226876784116E-4, 0.05),
        BACK_RIGHT (6, 5, 0.5, /*P:*/ 0.022237218916416168, /*I:*/ 6.017343257553875E-4, 0.05),
        FRONT_RIGHT(4, 3, 0,    /*P:*/ 0.03256119787693024, /*I:*/ 7.328314241021872E-4, 0.05),
        ;

        public final int drivingCANid, guidingCANid;
        public final double angleOffset;
        public final double drivingP, drivingI, guidingP;
        MotorPosition(int drivingCANid, int guidingCANid, double angleOffset, double drivingP, double drivingI, double guidingP){
            this.drivingCANid = drivingCANid;
            this.guidingCANid = guidingCANid;
            this.angleOffset = angleOffset * Math.PI;
            this.drivingP = drivingP;
            this.drivingI = drivingI;
            this.guidingP = guidingP;
        }
    }
}
