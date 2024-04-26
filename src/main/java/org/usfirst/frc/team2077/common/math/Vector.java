package org.usfirst.frc.team2077.common.math;

import org.usfirst.frc.team2077.common.VelocityDirection;

import java.util.EnumMap;

import static org.usfirst.frc.team2077.common.VelocityDirection.*;

//This should suffice until we manage to build a robot with more than 3 degrees of freedom
public class Vector extends EnumMap<VelocityDirection, Double> {

    public Vector() {
        super(VelocityDirection.class);
        put(FORWARD,    0.0);
        put(STRAFE,     0.0);
        put(ROTATION,   0.0);
    }

    public Vector(double forward, double strafe, double rotation) {
        super(VelocityDirection.class);
        put(FORWARD,    forward);
        put(STRAFE,     strafe);
        put(ROTATION,   rotation);
    }

    public Vector(Vector v){
        super(VelocityDirection.class);
        put(FORWARD,    v.get(FORWARD));
        put(STRAFE,     v.get(STRAFE));
        put(ROTATION,   v.get(ROTATION));
    }

    public double getMagnitude() {
        return Math.hypot(get(FORWARD), get(STRAFE));
    }

    public double getDirection(){
        return Math.atan2(get(FORWARD), get(STRAFE));
    }

    public double dot(Vector b){
        return get(FORWARD) * b.get(FORWARD) + get(STRAFE) * b.get(STRAFE);
    }

    public boolean isZero(){
        return (
            get(FORWARD) == 0 &&
            get(STRAFE) == 0 &&
            get(ROTATION) == 0
        );
    }

    public void add(Vector a){
        compute(FORWARD,    (k, v) -> v + a.get(FORWARD));
        compute(STRAFE,     (k, v) -> v + a.get(STRAFE));
    }

    public void rotate(double angle){
        double forward = get(STRAFE) * Math.sin(angle) + get(FORWARD) * Math.cos(angle);
        double strafe  = get(STRAFE) * Math.cos(angle) - get(FORWARD) * Math.sin(angle);

        put(FORWARD, forward);
        put(STRAFE, strafe);
    }

    public void scale(double s){
        compute(FORWARD, (k, v) -> v * s);
        compute(STRAFE,  (k, v) -> v * s);
    }

    public void normalize(){
        if(isZero()) return;

        scale(1 / getMagnitude());
    }

    public void flip(VelocityDirection axis){
        compute(axis, (k, v) -> v * -1);
    }

    public Vector copy(){
        return new Vector(this);
    }

}
