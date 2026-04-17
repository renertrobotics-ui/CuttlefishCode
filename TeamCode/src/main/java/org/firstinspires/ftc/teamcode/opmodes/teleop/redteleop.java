package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red Teleop")
public class redteleop extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public MotorEx transferMotor;

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

    public redteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static boolean red;
    public static boolean isRed(){
        return red;
    }

    public static int tagID;
    public static boolean findMotif = false;
    public boolean liftmid;
    boolean loweranglemid = false;
    public boolean shoot;
    private static final int APRILTAG_PIPELINE = 7;

    public boolean isInLaunchZone(double x, double y) {
        // Triangle 1: Goal Side (Top)
        if (y >= 64 && y <= 144) {
            double halfWidth = (y - 64);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }

        // Triangle 2: Audience Side (Bottom)
        if (y >= 0 && y <= 32) {
            double halfWidth = (32 - y);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }
        return false;
    }

    // --- NEW METHOD: Calculate Distance ---
    private double getDistanceToGoal() {
        Pose currentPose = follower.getPose();
        double deltaX = GOAL_X - currentPose.getX();
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
        red = true;
        intakeMotor = new MotorEx("Intake_Motor");
        transferMotor = new MotorEx("Transfer_Motor").reversed();

        Gamepads.gamepad2().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(()-> transferMotor.setPower(1))
                .whenBecomesFalse(() -> transferMotor.setPower(0));
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()->intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad2().rightTrigger().greaterThan(0.3).whenBecomesTrue(()-> transferMotor.setPower(-1))
                .whenBecomesFalse(() -> transferMotor.setPower(0));
        Gamepads.gamepad1().x().whenBecomesTrue(()->follower.setPose(new Pose(79.967,9.271,Math.toRadians(90))));
    }

    @Override
    public void onUpdate() {
        // 1. Find distance to the goal
        double distance = getDistanceToGoal();

        // 2. Plug distance into your regression formula
        float newtps = calculateShooterTPS(distance);

        // 3. Command the shooter
        shooter(newtps);

        // 4. Output telemetry for debugging
        ActiveOpMode.telemetry().addData("Distance to Goal (in)", distance);
        ActiveOpMode.telemetry().addData("Target TPS", newtps);
    }

    @Override
    public void onStartButtonPressed() {
        // Your start button logic goes here
    }

    public void onStop(){
        red = false;
    }
}