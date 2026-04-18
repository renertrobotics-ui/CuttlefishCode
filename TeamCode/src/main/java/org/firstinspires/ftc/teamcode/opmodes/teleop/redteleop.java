package org.firstinspires.ftc.teamcode.opmodes.teleop;
/*
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;
*/
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

import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.UpdateColorSensors;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.autonomousIntakeTransferOperation;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
/*
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalization;
import org.firstinspires.ftc.teamcode.subsystems.PositionalHood;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;
*/
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.calculate_heading;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.turret_on_via_encoder_and_crservos;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red Teleop")
public class redteleop extends NextFTCOpMode {

    public redteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveSubsystem.INSTANCE, ShooterSubsystem.INSTANCE, IntakeTransferSubsystem.INSTANCE, TurretSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean red;
    public static ElapsedTime runtime = new ElapsedTime();

    //public static ServoEx blocker = new ServoEx("blocker");
    public static boolean isRed(){
        return red;
    }

    public static int tagID;
    private boolean shooting = false;
    public static boolean findMotif = false;
    boolean firsttime = true;
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






    public boolean liftmid;
    boolean loweranglemid = false;

    public boolean isInLaunchZone(double x, double y) {

        // Triangle 1: Goal Side (Top)
        // Vertices: (-8, 144), (152, 144), (72, 64)
        // This triangle exists between y = 64 and y = 144.
        if (y >= 64 && y <= 144) {
            // As y increases from 64 to 144, the width of the triangle increases.
            // The slope of the edges is (144 - 64) / (152 - 72) = 80 / 80 = 1.
            double halfWidth = (y - 64);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }

        // Triangle 2: Audience Side (Bottom)
        // Vertices: (72, 32), (104, 0), (40, 0)
        // This triangle exists between y = 0 and y = 32.
        if (y >= 0 && y <= 32) {
            // As y decreases from 32 to 0, the width increases.
            // The slope of the edges is (32 - 0) / (72 - 40) = 32 / 32 = 1.
            double halfWidth = (32 - y);
            if (x >= (72 - halfWidth) && x <= (72 + halfWidth)) {
                return true;
            }
        }

        return false;
    }

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


    private static final int APRILTAG_PIPELINE = 7;
    @Override
    public void onInit() {
        red=true;
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> shooting = true)
                .whenBecomesFalse(() -> shooting = false);

        /*Gamepads.gamepad1().a().whenBecomesTrue(() -> blocker.setPosition(0.5));
        Gamepads.gamepad1().b().whenBecomesTrue(() -> blocker.setPosition(0.7));
        Gamepads.gamepad1().x().whenBecomesTrue(() -> blocker.setPosition(0.9));
        Gamepads.gamepad1().y().whenBecomesTrue(() -> blocker.setPosition(0.3));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> blocker.setPosition(0.1));*/


    }

    @Override
    public void onUpdate() {
        follower.update();
        Pose currPose = follower.getPose();
        double distance = getDistanceToGoal();

        // 2. Plug distance into your regression formula
        float newtps = calculateShooterTPS(distance);

        // 3. Command the shooter
        shooter(1800);

        double angle = calculate_heading(currPose);
        UpdateColorSensors();
        autonomousIntakeTransferOperation(shooting);
        turret_on_via_encoder_and_crservos(angle);
        //ActiveOpMode.telemetry().addData("position left", flywheel.getVelocity());
        //ActiveOpMode.telemetry().addData("position right", flywheel.getVelocity());
        ActiveOpMode.telemetry().update();



        /*
        if(lowerangle==true){
            newtps = findTPS44(DistanceRed.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        else if(lowerangle==false) {
            newtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
            shooter(newtps);
            //ActiveOpMode.telemetry().addData("newtps", newtps);
        }*/
    }

    public boolean shoot;

    @Override
    public void onStartButtonPressed() {


        //Gamepads.gamepad2().cross().whenBecomesTrue(() -> hood());
        //Gamepads.gamepad2().triangle().whenBecomesTrue(() -> hoodMid());
        /*SequentialGroup onStart= new SequentialGroup(
                new Delay(2),
                //TempHood.INSTANCE.HoodUp,
                new SetPower(transfer, 0.25),
                new Delay(0.01),
                new SetPower(transfer, 0),
                TempHood.INSTANCE.HoodUp,
                new SetPower(transfer, 1),
                new Delay(0.5),
                TempHood.INSTANCE.HoodDown,
                new SetPower(transfer, 0)
        );
        //int tag=MotifScanning.INSTANCE.findMotif();
        onStart.schedule();*/
    }


    public void onStop(){
        red=false;
    }
}