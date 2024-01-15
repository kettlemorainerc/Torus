/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FRC Team 2077. All Rights Reserved.                     */
/* Open Source Software - may be modified and shared by FRC teams.            */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2077;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.button.*;
import org.usfirst.frc.team2077.command.SparkMaxPIDTuner;
import org.usfirst.frc.team2077.common.command.*;
import org.usfirst.frc.team2077.common.control.DriveJoystick;
import org.usfirst.frc.team2077.common.control.DriveStick;
import org.usfirst.frc.team2077.common.control.DriveXboxController;
import org.usfirst.frc.team2077.subsystem.SwerveModule;

import java.util.ArrayList;
import java.util.stream.Collectors;

/**
 * This class is intended to be the center point of defining actions that can be utilized during teleop segments of
 * control. This is where we should define what USB port joysticks should be registered as in `FRC Driver Station`'s usb
 * menu. As well as define what buttons on primary/technical driver's controllers should do what.
 * */
public class DriveStation {
    // Common controller port numbers
    // Joysticks that support rotation
    private static final int DRIVE_JOYSTICK_PORT = 0;
    private static final int DRIVE_XBOX_PORT = 1;
    private static final int FLYSKY_PORT = 2;

    private static final int TECHNICAL_XBOX_PORT = 2;

    // Joysticks that do not support rotation
    private static final int TECHNICAL_JOYSTICK_PORT = 4;
    private static final int NUMPAD_PORT = 5;

    private final DriveXboxController driveStick;
    private final Joystick technicalStick;

    public DriveStation() {
        /** Set the driver's control method this MUST be a {@link DriveStick} implementation */
//        driveStick = getFlysky();
//        driveStick = getJoystick();
        driveStick = getXbox(DRIVE_XBOX_PORT);

        /** Set the technical control method. This can be any {@link Joystick} implementation */
//        technicalStick = getTechnicalJoystick();
//        technicalStick = getXbox(TECHNICAL_XBOX_PORT);
        technicalStick = getNumpad();

        bind();
    }

    /**
     * This method binds any subsystem's default command and bind commands to a user's chosen
     * control method.
     */
    public void bind() {
        RobotHardware hardware = RobotHardware.getInstance();

        hardware.getPosition().setDefaultCommand(new CardinalMovement(driveStick));
        hardware.getHeading().setDefaultCommand(new RotationMovement(driveStick));

        bindDriverControl(driveStick);
        bindTechnicalControl(technicalStick);
    }

    /** Bind primary driver's button commands here */
    private static void bindDriverControl(DriveXboxController primary) {
//        primary.getRightTriggerAxis()
    }

    /** Bind technical driver button commands here */
    private void bindTechnicalControl(Joystick secondary) {
//        InputMap.bindAxis(Claw.Input.CLOSE, secondary::getRightTriggerAxis);
//        InputMap.bindAxis(ScissorArm.Input.EXTEND, secondary::getLeftY);

//        new PIDAutoTune().bind(new JoystickButton(secondary, 1));

        ArrayList<SwerveModule> modules = new ArrayList<>(RobotHardware.getInstance().getChassis().getDriveModules().values());
        ArrayList<SparkMaxPIDController> pids = new ArrayList<>(modules.stream().map(SwerveModule::getGuidingPID).collect(Collectors.toList()));

//        for(int i = 0; i < 4; i++) {
//            new SparkMaxPIDTuner<>(
//                    modules.subList(i, i + 1),
//                    pids.subList(i, i + 1),
//                    SwerveModule::calibrationSetVelocity,
//                    SwerveModule::getVelocityMeasured,
//                    SwerveModule::savePID,
//                    0.08658034, 0.00090362,
//                    3.25, 4,
//                    SparkMaxPIDTuner.ErrorMethod.DIFFERENCE,
//                    15
//            ).bind(new JoystickButton(secondary, 1));
//        }

        for(int i = 0; i < 4; i++) {
            new SparkMaxPIDTuner<>(
                    modules.subList(i, i + 1),
                    pids.subList(i, i + 1),
                    SwerveModule::calibrationSetAngle,
                    SwerveModule::getAngle,
                    SwerveModule::savePID,
                    0.12235982, 0.00003915,
                    Math.PI / 2.0, 4,
                    SparkMaxPIDTuner.ErrorMethod.ANGLE_DIFFERENCE,
                    20,
                    new JoystickButton(secondary, 2)
            ).bind(new JoystickButton(secondary, 1));
        }

//        new SparkMaxPIDTuner<>(
//            modules, pids,
//            SwerveModule::calibrationSetAngle,
//            SwerveModule::getAngle,
//            SwerveModule::savePID,
//            0.10372401, 0.00005248,
//            Math.PI / 2.0, 4,
//            SparkMaxPIDTuner.ErrorMethod.ANGLE_DIFFERENCE,
//            10
//        ).bind(new JoystickButton(secondary, 1));
    }

    /** Normal (silver/brighter) joystick that supports rotation */
    private static DriveJoystick getJoystick() {
        return new DriveJoystick(DRIVE_JOYSTICK_PORT).setDriveSensitivity(.15, 5)
                                                     .setRotationSensitivity(.1, 1);
    }

    /** Flysky Drone Controller */
    private static DriveJoystick getFlysky() {
        return new DriveJoystick(FLYSKY_PORT, 4).setDriveSensitivity(.3, 1)
                                                .setRotationSensitivity(.05, 2.5);
    }

    private static DriveXboxController getXbox(int port){
        return new DriveXboxController(port).setDriveSensitivity(.3,1)
                                                       .setRotationSensitivity(.3,1.5);
    }

    /** Currently the darker joystick that doesn't support rotation */
    private static Joystick getTechnicalJoystick() {
        return new Joystick(TECHNICAL_JOYSTICK_PORT);
    }

    private static Joystick getNumpad() {
        return new Joystick(NUMPAD_PORT);
    }

    /** bind command to the given joystick button */
    public static void useCommand(Joystick joystick, int button, BindableCommand command) {
        command.bind(new JoystickButton(joystick, button));
    }
}
