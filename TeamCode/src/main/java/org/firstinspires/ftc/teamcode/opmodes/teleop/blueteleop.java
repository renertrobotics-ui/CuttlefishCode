package org.firstinspires.ftc.teamcode.opmodes.teleop; /*

import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;*/

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants; /*
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;*/
import org.firstinspires.ftc.teamcode.subsystems.drivesubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakeTransferSubsystem;


import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "blue teleop")
public class blueteleop extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public MotorEx transferMotor;

    public blueteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(drivesubsystem.INSTANCE, intakeTransferSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE


        );
    }

    public static boolean blue;
    static ColorSensor Intake;
    static ColorSensor Transfer;
    public static boolean isBlue(){
        return blue;
    }




    private static final int APRILTAG_PIPELINE = 8;
    @Override
    public void onInit() {
        blue=true;




    }

    @Override
    public void onUpdate() {
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