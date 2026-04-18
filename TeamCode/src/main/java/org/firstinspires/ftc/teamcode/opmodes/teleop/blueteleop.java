package org.firstinspires.ftc.teamcode.opmodes.teleop; /*

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;*/

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.UpdateColorSensors;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.autonomousIntakeTransferOperation;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.calculate_heading;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.turret_on_via_encoder_and_crservos;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants; /*
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;*/
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;


import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "blue teleop")
public class blueteleop extends NextFTCOpMode {

    public blueteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveSubsystem.INSTANCE, IntakeTransferSubsystem.INSTANCE, TurretSubsystem.INSTANCE, ShooterSubsystem.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean blue;
    public static ElapsedTime newtime = new ElapsedTime();

    private boolean shooting = false;
    public static boolean isBlue(){
        return blue;
    }


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
//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(APRILTAG_PIPELINE);
//        limelight.start();
        blue=true;

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> shooting = true)
                .whenBecomesFalse(() -> shooting = false);



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
        turret_on_via_encoder_and_crservos(angle);
        autonomousIntakeTransferOperation(shooting);
        //float newtps=100
        //turret_on_via_encoder_and_crservos(-10000);
        //float newtps=1000;
        //shooter(newtps);
        //ActiveOpMode.telemetry().addData("newtps", newtps);
/*if(lowerangle==true){
            newtps = findTPS44(DistanceBlue.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        else if(lowerangle==false) {
            newtps = findTPS(DistanceBlue.INSTANCE.getDistanceFromTag());
            //ActiveOpMode.telemetry().addData("Lowerangle:", lowerangle);
        }
        if (DistanceBlue.INSTANCE.getDistanceFromTag() != 0) {
            //shooter(newtps);
            ActiveOpMode.telemetry().addData("newtps", newtps);
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
                new SetPower(transferMotor, 0.25),
                new Delay(0.01),
                new SetPower(transferMotor, 0),
                TempHood.INSTANCE.HoodUp,
                new SetPower(transferMotor, 1),
                new Delay(0.5),
                TempHood.INSTANCE.HoodDown,
                new SetPower(transferMotor, 0)
        );
        //int tag=MotifScanning.INSTANCE.findMotif();
        onStart.schedule();*/
    }


    public void onStop(){
        blue=false;
    }
}