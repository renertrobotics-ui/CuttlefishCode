package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.drivesubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "blue teleop")
public class blueteleop extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    private double targetVelocity;

    public blueteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                // Added ShooterSubsystem.INSTANCE here so its periodic() loop runs
                new SubsystemComponent(drivesubsystem.INSTANCE, ShooterSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static boolean blue;
    public static boolean isBlue() { return blue; }

    @Override
    public void onInit() {
        blue = true;
        intakeMotor = new MotorEx("Intake_Motor");
        transferMotor = new MotorEx("Transfer_Motor");

        // --- Shooter Controls ---
        // Sets velocity to 2000 when A (Cross) is pressed
        Gamepads.gamepad1().a().whenBecomesTrue(() -> ShooterSubsystem.INSTANCE.setVelocity(2000));

        // Optional: Sets velocity to 0 when B (Circle) is pressed to stop the flywheels
        Gamepads.gamepad1().b().whenBecomesTrue(() -> ShooterSubsystem.INSTANCE.setVelocity(0));

        // --- Existing Intake/Transfer Controls ---
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(() -> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> transferMotor.setPower(1))
                .whenBecomesFalse(() -> transferMotor.setPower(0));

        Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(() -> transferMotor.setPower(-1))
                .whenBecomesFalse(() -> transferMotor.setPower(0));
    }

    @Override
    public void onUpdate() {
        // Telemetry to track shooter speed
        targetVelocity = 1400;
        shooter(targetVelocity);
        telemetry.addData("Target Velocity", 2000);
        telemetry.addData("Left Flywheel Vel", ShooterSubsystem.flywheelvelocity);
        telemetry.addData("Right Flywheel Vel", ShooterSubsystem.flywheelvelocity2);
    }

    @Override
    public void onStartButtonPressed() {
    }

    public void onStop() {
        blue = false;
        // Safety: Stop shooter when OpMode stops
        targetVelocity = 0;
    }
}