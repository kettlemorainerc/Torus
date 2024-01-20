package org.usfirst.frc.team2077.math;

import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.control.DriveStick;
import org.usfirst.frc.team2077.common.math.Matrix;
import org.usfirst.frc.team2077.drivetrain.SwerveModuleIF;

import java.util.EnumMap;
import java.util.Map;
import java.util.Objects;

import static java.lang.Math.*;
import static org.usfirst.frc.team2077.common.VelocityDirection.*;
import static org.usfirst.frc.team2077.common.WheelPosition.*;

/**
 * Handle calculating the necessary magnitude and angle for a set of swerve wheels.
 * We may want to find a better equation or come up with our own we like better.
 * <p>
 *  Inverse kinematics based on pdf found <a href="https://www.chiefdelphi.com/uploads/default/original/3X/e/f/ef10db45f7d65f6d4da874cd26db294c7ad469bb.pdf">here</a>.
 * </p>
 * <p>
 *   Forward kinematics from <a href="https://ietresearch.onlinelibrary.wiley.com/doi/10.1049/joe.2014.0241">this paper</a>
 *   <p>
 *     Worth noting is that I'm unsure if the forward kinematics equation used actually has a hard requirement of
 *     rectangularly positioned wheels.
 *   </p>
 * </p>
 *
 * <dl>
 *     <dt>Y</dt>
 *     <dd>Equates to the value of the {@link DriveStick}'s north value. May need to be inverted</dd>
 *     <dt>X</dt>
 *     <dd>Equates to the value of the {@link DriveStick}'s east value. Shouldn't need to be inverted</dd>
 *     <dt>Z</dt>
 *     <dd>Equations the value of the {@link DriveStick}'s rotation value.</dd>
 *     <dt>L - (wheelbase)</dt>
 *     <dt>Length between the center of a front wheel and the center of the back wheel on the same side</dt>
 *     <dt>W - (trackWidth)</dt>
 *     <dt>Width between the left and right wheels in the front/back</dt>
 * </dl>
 */
public class SwerveMath {
    private static final EnumMap<WheelPosition, Multiplier> WHEEL_MULTIPLIERS = new EnumMap<>(WheelPosition.class);
    private static final EnumMap<WheelPosition, JointKey> WHEEL_TO_SIDES = new EnumMap<>(WheelPosition.class);
    static {
        WHEEL_TO_SIDES.put(FRONT_LEFT, new JointKey(RobotSide.FRONT, RobotSide.LEFT));
        WHEEL_TO_SIDES.put(BACK_LEFT, new JointKey(RobotSide.BACK, RobotSide.LEFT));
        WHEEL_MULTIPLIERS.put(FRONT_LEFT, new Multiplier(-1, 1));
        WHEEL_MULTIPLIERS.put(BACK_LEFT, new Multiplier(-1, -1));

        WHEEL_TO_SIDES.put(FRONT_RIGHT, new JointKey(RobotSide.FRONT, RobotSide.RIGHT));
        WHEEL_TO_SIDES.put(BACK_RIGHT, new JointKey(RobotSide.BACK, RobotSide.RIGHT));
        WHEEL_MULTIPLIERS.put(FRONT_RIGHT, new Multiplier(1, 1));
        WHEEL_MULTIPLIERS.put(BACK_RIGHT, new Multiplier(1, -1));
    }

    private static double pythag(double a, double b) {
        return sqrt(pow(a, 2) + pow(b, 2));
    }

    private double wheelbase, trackWidth, radius;
    private double halfWheelbase, halfTrackwidth, wDenom;

    public SwerveMath(double wheelbase, double trackWidth) {
        setWheelbase(wheelbase);
        setTrackWidth(trackWidth);
    }

    public void setWheelbase(double wheelbase) {
        this.wheelbase = wheelbase;
        this.halfWheelbase = wheelbase / 2;
        updateRadius();
    }

    public void setTrackWidth(double trackWidth) {
        this.trackWidth = trackWidth;
        this.halfTrackwidth = trackWidth / 2;
        updateRadius();
    }

    private void updateRadius() {
        this.radius = pythag(wheelbase, trackWidth);
        this.wDenom = pow(wheelbase, 2) + pow(trackWidth, 2);
    }

    private EnumMap<RobotSide, Double> createRobotSideValueMap(
            double forward, double strafe, double rotation
    ) {
        EnumMap<RobotSide, Double> multipliers = new EnumMap<>(RobotSide.class);

        // A
        multipliers.put(RobotSide.BACK, strafe - rotation * wheelbase / radius);
        // B
        multipliers.put(RobotSide.FRONT, strafe + rotation * wheelbase / radius);
        // C
        multipliers.put(RobotSide.RIGHT, forward - rotation * trackWidth / radius);
        // D
        multipliers.put(RobotSide.LEFT, forward + rotation * trackWidth / radius);

        return multipliers;
    }

    private SwerveTargetValues wheelTargets(
            Map<RobotSide, Double> values,
            RobotSide east,
            RobotSide north
    ) {
        double mag = Math.sqrt(pow(values.get(east), 2) + pow(values.get(north), 2));
        double ang = atan2(values.get(north), values.get(east));

        if(Double.isNaN(mag)) mag = 0;
        if(Double.isNaN(ang)) ang = 0;
        else if(ang < 0) {
            ang += 2 * Math.PI;
        }

        return new SwerveTargetValues(mag, ang);
    }

    public Map<WheelPosition, SwerveTargetValues> targetsForVelocities(
            Map<VelocityDirection, Double> targetMagnitudes
    ) {
        double north = targetMagnitudes.get(FORWARD);
        double strafe = targetMagnitudes.get(STRAFE);
        double rotation = targetMagnitudes.get(ROTATION);

        if(rotation == 0 && north == 0 && strafe == 0) {
            return Map.of(
                    FRONT_LEFT, new SwerveTargetValues(0, 0),
                    FRONT_RIGHT, new SwerveTargetValues(0, 0),
                    BACK_LEFT, new SwerveTargetValues(0, 0),
                    BACK_RIGHT, new SwerveTargetValues(0, 0)
            );
        }

        // Some mix of north/strafe/rotation
        Map<RobotSide, Double> valueMap = createRobotSideValueMap(north, strafe, rotation);

        Map<WheelPosition, SwerveTargetValues> targetValues = new EnumMap<>(WheelPosition.class);

        WHEEL_TO_SIDES.forEach((position, sides) -> targetValues.put(position, wheelTargets(valueMap, sides.east, sides.north)));

        double max = targetValues.values().stream().mapToDouble(SwerveTargetValues::getMagnitude).max().orElse(0d);
        if(max > 1) targetValues.values().forEach(val -> val.setMagnitude(val.getMagnitude() / max));

        return targetValues;
    }

    public Map<WheelPosition, SwerveTargetValues> targetsForVelocities(
          Map<VelocityDirection, Double> targetMagnitudes,
          double maxSpeed, double maxRotation
    ) {
       return targetsForVelocities(Map.of(
             FORWARD, targetMagnitudes.get(FORWARD) / maxSpeed,
             STRAFE, targetMagnitudes.get(STRAFE) / maxSpeed,
             ROTATION, targetMagnitudes.get(ROTATION) / maxRotation
       ));
    }

    public Map<WheelPosition, SwerveTargetValues> targetsForVelocities(
            Map<VelocityDirection, Double> targetMagnitudes,
            double maxSpeed, double maxRotation, double angleOffset
    ){
        return targetsForVelocities(rotateTargets(targetMagnitudes, angleOffset), maxSpeed, maxRotation);
    }

    public Map<VelocityDirection, Double> rotateTargets(
            Map<VelocityDirection, Double> tar, double angle
    ){
        Map<VelocityDirection, Double> copy = new EnumMap(tar);

        copy.put(FORWARD,Math.sin(angle) * tar.get(STRAFE) + Math.cos(angle) * tar.get(FORWARD));
        copy.put(STRAFE, Math.cos(angle) * tar.get(STRAFE) - Math.sin(angle) * tar.get(FORWARD));

        return copy;
    }

    /**
     * Handles solving for <strong>W<sub>i</sub></strong>.<br>
     * In regards to<br>
     * <code>
     *     W<sub>i</sub> = (-y<sup>r</sup><sub>wi</sub>cos(&delta;<sub>i</sub>) + x<sup>r</sup><sub>wi</sub>sin(&delta;<sub>i</sub>)) / 4((x<sup>r</sup><sub>wi</sub>)<sup>2</sup> + (y<sup>r</sup><sub>wi</sub>)<sup>2</sup>)
     * </code>
     * <br><br>
     * Given <br>
     * "y<sup>r</sup><sub>wi</sub>" is the Y offset of wheel I<br>
     * "x<sup>r</sup><sub>wi</sub>" is the X offset of wheel I<br>
     * "&delta;<sub>i</sub>" is the Angle wheel I<br>
     *
     * @param p The position we're calculating for
     * @param cos cos(&delta;<sub>i</sub>)
     * @param sin sin(&delta;<sub>i</sub>)
     * @return W<sub>i</sub>
     */
    private double getWFor(WheelPosition p, double cos, double sin) {
        var mults = WHEEL_MULTIPLIERS.get(p);

        return (
                ((mults.y * -halfWheelbase * cos) + (mults.x * halfTrackwidth * sin)) /
                wDenom
        );
    }

    public Map<VelocityDirection, Double> velocitiesForTargets( //TODO: potentially rename to something regarding forward kinematics
        Map<WheelPosition, ? extends SwerveModuleIF> targets
    ) {
        SwerveModuleIF fl = targets.get(FRONT_LEFT);
        SwerveModuleIF bl = targets.get(BACK_LEFT);
        SwerveModuleIF br = targets.get(BACK_RIGHT);
        SwerveModuleIF fr = targets.get(FRONT_RIGHT);

        // We have to convert our wheel angles to their unit circle equivalent
        // We go 0 (north) - 360 clockwise
        // The unit circle goes 0 (east) - 360 counter-clockwise
        // So -angle inverts to counter-clockwise
        // and + 90 (un)adjusts to make north 0
//        double flA = toRadians(-fl.getAngle() + 90),
//                blA = toRadians(-bl.getAngle() + 90),
//                brA = toRadians(-br.getAngle() + 90),
//                frA = toRadians(-fr.getAngle() + 90);

        double flA = fl.getAngle(),
                blA = bl.getAngle(),
                brA = br.getAngle(),
                frA = fr.getAngle();

        double flC = cos(flA), flS = sin(flA);
        double blC = cos(blA), blS = sin(blA);
        double brC = cos(brA), brS = sin(brA);
        double frC = cos(frA), frS = sin(frA);

        Matrix pseudoPDotX = new Matrix(new double[][] {
                {flC / 4, blC / 4, brC / 4, frC / 4},
                {flS / 4, blS / 4, brS / 4, frS / 4},
                {getWFor(FRONT_LEFT, flC, flS), getWFor(BACK_LEFT, blC, blS), getWFor(BACK_RIGHT, brC, brS), getWFor(FRONT_RIGHT, frC, frS)},
        });

        double flV = fl.getVelocityMeasured(),
                blV = bl.getVelocityMeasured(),
                brV = br.getVelocityMeasured(),
                frV = fr.getVelocityMeasured();

        Matrix velocities = new Matrix(new double[][] {
                {flV},
                {blV},
                {brV},
                {frV},
        });

        Matrix result = pseudoPDotX.multiply(velocities);

        return Map.of(
                FORWARD,  result.get(0, 0),
                STRAFE,   result.get(0, 1),
                ROTATION, result.get(0, 2)
        );
    }

    enum RobotSide {
        LEFT(true), RIGHT(true), FRONT(false), BACK(false);
        public final boolean east;

        RobotSide(boolean east) {
            this.east = east;
        }
    }

    private static class JointKey {
        final RobotSide east;
        final RobotSide north;

        JointKey(RobotSide north, RobotSide east) {
            this.east = east;
            this.north = north;
        }

        @Override public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            JointKey jointKey = (JointKey) o;
            return east == jointKey.east && north == jointKey.north;
        }

        @Override public int hashCode() {
            return Objects.hash(east, north);
        }

        @Override public String toString() {
            return "JointKey{" +
                   "east=" + east +
                   ", north=" + north +
                   '}';
        }
    }

    private static class Multiplier {
        int x, y;
        Multiplier(int x, int y) {
            this.x = x;
            this.y = y;
        }
    }
}
