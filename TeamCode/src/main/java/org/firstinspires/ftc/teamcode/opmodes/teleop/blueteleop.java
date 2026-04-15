package org.firstinspires.ftc.teamcode.opmodes.teleop; /*

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;*/

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.UpdateColorSensors;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.autonomousIntakeTransferOperation;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.calculate_heading;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.turret_on_via_encoder_and_crservos;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants; /*
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;*/
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
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
                new SubsystemComponent(DriveSubsystem.INSTANCE, IntakeTransferSubsystem.INSTANCE, TurretSubsystem.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean blue;
    public static boolean isBlue(){
        return blue;
    }




    @Override
    public void onInit() {
//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.pipelineSwitch(APRILTAG_PIPELINE);
//        limelight.start();
        blue=true;




    }

    @Override
    public void onUpdate() {
        follower.update();
        Pose currPose = follower.getPose();

        float newtps=1000;
        double angle = calculate_heading(currPose);
        UpdateColorSensors();
        autonomousIntakeTransferOperation();
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