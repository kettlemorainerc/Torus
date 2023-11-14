/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077.common.drivetrain;

import org.usfirst.frc.team2077.common.Clock;
import org.usfirst.frc.team2077.common.VelocityDirection;
import org.usfirst.frc.team2077.common.WheelPosition;
import org.usfirst.frc.team2077.common.math.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.*;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

public abstract class AbstractChassis<DriveModule> extends SubsystemBase implements DriveChassisIF {

    private static <T> EnumMap<VelocityDirection, T> defaultedDirectionMap(T defaultValue) {
        EnumMap<VelocityDirection, T> newMap = new EnumMap<>(VelocityDirection.class);
        for (VelocityDirection d : VelocityDirection.values()) newMap.put(d, defaultValue);
        return newMap;
    }

    public final Map<WheelPosition, DriveModule> driveModules;

    protected double maximumSpeed;
    protected double maximumRotation;
    protected double minimumSpeed;
    protected double minimumRotation;

    protected double lastTime;
    protected double deltaTime;

    protected final Position position = new Position();

    protected Map<VelocityDirection, Double> velocitySet = defaultedDirectionMap(0d);
    protected Map<VelocityDirection, Double> velocityMeasured = defaultedDirectionMap(0d);

    public AbstractChassis(Map<WheelPosition, DriveModule> driveModules) {
        this.driveModules = driveModules;

        lastTime = Clock.getSeconds();
    }

    @Override public void periodic() {
        double now = Clock.getSeconds();
        deltaTime = now - lastTime;
        lastTime = now;

        measureVelocity();
        updatePosition();
        updateDriveModules();
    }

    public void updatePosition(){
        for(VelocityDirection axis : VelocityDirection.values()){
            position.move(velocityMeasured.get(axis) * deltaTime, axis);
        }
    }

    public Map<WheelPosition, DriveModule> getDriveModules(){
        return driveModules;
    }

    /**
     * Update drive module setpoints.
     * Called by {@link #periodic()}.
     */
    protected abstract void updateDriveModules();

    /**
     *  Updates velocity measured
     *  Called by {@Link #periodic()}.
     */
    protected abstract void measureVelocity();

    @Override public Map<VelocityDirection, Double> getVelocitySet() {
        return new EnumMap<>(velocitySet);
    }

    @Override public Map<VelocityDirection, Double> getVelocityMeasured() {
        return new EnumMap<>(velocityMeasured);
    }

    @Override public Map<VelocityDirection, Double> getMaximumVelocity() {
        Map<VelocityDirection, Double> max = new EnumMap<>(VelocityDirection.class);

        max.put(FORWARD, maximumSpeed);
        max.put(STRAFE, maximumSpeed);
        max.put(ROTATION, maximumRotation);

        return max;
    }

    @Override public Map<VelocityDirection, Double> getMinimumVelocity() {
        Map<VelocityDirection, Double> min = new EnumMap<>(VelocityDirection.class);

        min.put(FORWARD, minimumSpeed);
        min.put(STRAFE, minimumSpeed);
        min.put(ROTATION, minimumRotation);

        return min;
    }

    @Override public Position getPosition() {
        return position.copy();
    }

    @Override public void setVelocity(double forward, double strafe, double rotation){
        setVelocity(forward, strafe);
        setRotation(rotation);
    }

    @Override public void setVelocity(double forward, double strafe){
        velocitySet.put(FORWARD, forward);
        velocitySet.put(STRAFE, strafe);
    }

    @Override public void setRotation(double rotation){
        velocitySet.put(ROTATION, rotation);
    }

    @Override public final void setVelocityPercent(double forward, double strafe, double rotation) {
        setVelocityPercent(forward, strafe);
        setRotationPercent(rotation);
    }

    @Override public final void setVelocityPercent(double forward, double strafe) {
        setVelocity(forward * maximumSpeed, strafe * maximumSpeed);
    }

    @Override public final void setRotationPercent(double rotation) {
        setRotation(rotation * maximumRotation);
    }

    @Override public void halt() {
        setVelocity(0, 0, 0);
    }

    public DriveModule getWheel(WheelPosition position){
        return driveModules.get(position);
    }
}
