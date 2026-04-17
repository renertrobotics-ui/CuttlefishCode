package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

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
    // --- REGRESSION & TARGETING CONSTANTS ---
    private static final double GOAL_X = 141.5;
    private static final double GOAL_Y = 141.5;

    // Quartic Regression Coefficients (from image)
    private static final double a = -3.25025e-7;
    private static final double b = 0.000367273;
    private static final double c = -0.145764;
    private static final double d = 25.48778;

    // TODO: The 'e' value was cut off in your screenshot! Replace 0.0 with your actual 'e' value.
    private static final double e = -3.291;
    // ----------------------------------------


    public blueteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                // Added ShooterSubsystem.INSTANCE here so its periodic() loop runs
                new SubsystemComponent(DriveSubsystem.INSTANCE, ShooterSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static boolean blue;
    public static boolean isBlue() { return blue; }
    private double getDistanceToGoal() {
        Pose currentPose = follower.getPose();
        double deltaX = currentPose.getX();
        double deltaY = GOAL_Y - currentPose.getY();

        // Math.hypot calculates sqrt(deltaX^2 + deltaY^2)
        return 2.54*Math.hypot(deltaX, deltaY);
    }

    // --- NEW METHOD: Calculate Target TPS via Regression ---
    private float calculateShooterTPS(double distance) {
        double targetTPS = (a * Math.pow(distance, 4)) +
                (b * Math.pow(distance, 3)) +
                (c * Math.pow(distance, 2)) +
                (d * distance) + e;

        return (float) targetTPS;
    }

    @Override
    public void onInit() {
        blue = true;
        intakeMotor = new MotorEx("Intake_Motor");
        transferMotor = new MotorEx("Transfer_Motor");
/*
        // --- Shooter Controls ---
        // Sets velocity to 2000 when A (Cross) is pressed
        Gamepads.gamepad1().a().whenBecomesTrue(() -> ShooterSubsystem.INSTANCE.setVelocity(2000));

        // Optional: Sets velocity to 0 when B (Circle) is pressed to stop the flywheels
        Gamepads.gamepad1().b().whenBecomesTrue(() -> ShooterSubsystem.INSTANCE.setVelocity(0));


        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(() -> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> transferMotor.setPower(1))
                .whenBecomesFalse(() -> transferMotor.setPower(0));

        Gamepads.gamepad1().rightTrigger().greaterThan(0.3).whenBecomesTrue(() -> transferMotor.setPower(-1))
                .whenBecomesFalse(() -> transferMotor.setPower(0)); */
    }

    @Override
    public void onUpdate() {
        // Telemetry to track shooter speed
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