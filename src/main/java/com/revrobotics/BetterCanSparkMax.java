package com.revrobotics;

public class BetterCanSparkMax extends CANSparkMax {
    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected
     *                 to their matching colors and the hall sensor must be plugged in. Brushed motors must be
     *                 connected to the Red and Black terminals only.
     */
    public BetterCanSparkMax(
          int deviceId,
          MotorType type
    ) {
        super(deviceId, type);
    }

    public void setTargetVelocity(double velocity) {
        setpointCommand(velocity, ControlType.kVelocity);
    }

}
