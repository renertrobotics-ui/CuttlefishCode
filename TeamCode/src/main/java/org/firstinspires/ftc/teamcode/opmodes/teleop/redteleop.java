package org.firstinspires.ftc.teamcode.opmodes.teleop;
/*
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;
*/
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.UpdateColorSensors;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.autonomousIntakeTransferOperation;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;

import com.pedropathing.geometry.Pose;

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


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red Teleop")
public class redteleop extends NextFTCOpMode {

    public redteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(DriveSubsystem.INSTANCE, IntakeTransferSubsystem.INSTANCE, TurretSubsystem.INSTANCE),
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




    private static final int APRILTAG_PIPELINE = 7;
    @Override
    public void onInit() {
        red=true;







    }

    @Override
    public void onUpdate() {
        follower.update();
        Pose currPose = follower.getPose();

        float newtps=1000;
        double angle = calculate_heading(currPose);
        UpdateColorSensors();
        autonomousIntakeTransferOperation();

        //turret_on_via_encoder_and_crservos(angle);



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