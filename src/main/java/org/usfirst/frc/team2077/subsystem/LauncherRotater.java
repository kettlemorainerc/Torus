package org.usfirst.frc.team2077.subsystem;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Subsystem;

    public class LauncherRotater implements Subsystem {
        //TODO: determine device number & speed
        private TalonSRX RotationControllerMotor = new TalonSRX(0);
        private TalonSRX RotaryEncoder = new TalonSRX(0);
        private double currentAngle;
        private double targetAngle;
        private double deadZone =.05;

        public enum InputDir{
            FORWARD,
            BACKWARD,
            FRONT
        }

        public LauncherRotater() {

        }
        @Override
        public void periodic() {
            currentAngle = RotaryEncoder.getSensorCollection().getPulseWidthPosition();
            GoToTarget(targetAngle);
        }

        public void RotateToIntake() {
            targetAngle = (2.0/3.0)*Math.PI;
        }
        public void RotateForward(){
            targetAngle += .1;
        }
        public void RotateBackward(){
            targetAngle -= .1;
        }

        public void GoToTarget(double targetAngle){
            RotationControllerMotor.set(TalonSRXControlMode.PercentOutput, targetAngle/Math.abs(currentAngle));
        }




    }
