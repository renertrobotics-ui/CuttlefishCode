package org.firstinspires.ftc.teamcode.opmodes.teleop;
/*
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;
*/
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
/*
import org.firstinspires.ftc.teamcode.subsystems.DistanceBlue;
import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
import org.firstinspires.ftc.teamcode.subsystems.LimelightLocalization;
import org.firstinspires.ftc.teamcode.subsystems.PositionalHood;
import org.firstinspires.ftc.teamcode.subsystems.TempHood;
*/
import org.firstinspires.ftc.teamcode.subsystems.drivesubsystem;


import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.Drivetrain;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Red Teleop")
public class redteleop extends NextFTCOpMode {

    public MotorEx intakeMotor;
    public MotorEx transferMotor;
    public redteleop() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(drivesubsystem.INSTANCE/*, Intake.INSTANCE, Spindexer.INSTANCE*/),
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
        intakeMotor = new MotorEx("Intake_Motor").reversed();
        transferMotor = new MotorEx("Transfer_Motor").reversed();
        Gamepads.gamepad1().leftTrigger().greaterThan(0.3).whenBecomesTrue(()-> intakeMotor.setPower(1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(()-> transferMotor.setPower(1))
                .whenBecomesFalse(() -> transferMotor.setPower(0));
        Gamepads.gamepad2().leftTrigger().greaterThan(0.3).whenBecomesTrue(()->intakeMotor.setPower(-1))
                .whenBecomesFalse(() -> intakeMotor.setPower(0));
        Gamepads.gamepad2().rightTrigger().greaterThan(0.3).whenBecomesTrue(()-> transferMotor.setPower(-1))
                .whenBecomesFalse(() -> transferMotor.setPower(0));
        Gamepads.gamepad1().x().whenBecomesTrue(()->follower.setPose(new Pose(79.967,9.271,Math.toRadians(90))));







    }

    @Override
    public void onUpdate() {
        float newtps=1000;
        shooter(newtps);
        ActiveOpMode.telemetry().addData("newtps", newtps);

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