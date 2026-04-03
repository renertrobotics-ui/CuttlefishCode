package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit.DEGREES;
//import static org.firstinspires.ftc.teamcode.opModes.TeleOp.FarzoneTeleOpBlue.isBlueFar;
//import static org.firstinspires.ftc.teamcode.opModes.TeleOp.FarzoneTeleOpRed.isRedFar;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
//import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
//import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS44;
//import static org.firstinspires.ftc.teamcode.subsystems.Calculations.lowangle;
//import static org.firstinspires.ftc.teamcode.subsystems.Flywheel.shooter;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.redteleop.isRed;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;

import static org.firstinspires.ftc.teamcode.opmodes.teleop.blueteleop.isBlue;

//import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;
//import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;
import dev.nextftc.hardware.powerable.SetPower;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;


@Configurable
public class drivesubsystem implements Subsystem {

    public static final drivesubsystem INSTANCE = new drivesubsystem();
    public drivesubsystem() {
    }


    public double aimMultiplier = 0.575;

    private boolean slow = false;
    // ADDED: Fields to track error over time for D term
    private double lastError = 0;
    private double lastTime = 0;


    public static final MotorEx fL = new MotorEx("Front left drive motor BRFL").brakeMode().reversed();
    public static final MotorEx bL = new MotorEx("back left drive motor BLORF").brakeMode().reversed();
    public static final MotorEx fR = new MotorEx("Front right drive motor BLORF").brakeMode().reversed();
    public static final MotorEx bR = new MotorEx("back right drive motor BRFL").brakeMode().reversed();

    private IMUEx imu;

    public boolean firsttime = true;

    public int alliance;




    Pose startingpose = Storage.currentPose;
    @Override
    public Command getDefaultCommand() {

        if (isBlue() != true && isRed() != true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        } else {
            if (isBlue() == true) {
                alliance = 1;
                }
            }
            if (isRed() == true) {
                alliance = -1;
            }

        follower.update();
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
            //driving stuff here for mecanum
            if (slow == true) {
                return new MecanumDriverControlled(
                        fL,
                        fR,
                        bL,
                        bR,
                        Gamepads.gamepad1().leftStickX().map(it -> alliance * it * 0.4),
                        Gamepads.gamepad1().leftStickY().map(it -> alliance * it *-0.4),
                        Gamepads.gamepad1().rightStickX().map(it -> it * 0.4 * -0.75),
                        new FieldCentric(imu)
                );
            }
            else //IF SLOW IS OFF
            {
                return new MecanumDriverControlled(
                        fL,
                        fR,
                        bL,
                        bR,
                        Gamepads.gamepad1().leftStickX().map(it -> 1.5 *alliance *it),
                        Gamepads.gamepad1().leftStickY().map(it -> alliance * -1.5 * it),
                        Gamepads.gamepad1().rightStickX().map(it -> it * -1),
                        new FieldCentric(imu)
                );
            }
        }


    public Command localize;



    @Override
    public void initialize() {

        firsttime = true;
        follower = follower();
        if(isBlue()!=true && isRed()!=true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        }
        //alliance 1 means on blue alliance, -1 means on red alliance
        else{
            if(isBlue()==true) {
                alliance=1;
            }
            if(isRed()==true){
                alliance=-1;
            }
            }
        imu = new IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed();

        if(alliance ==-1){
            startingpose = new Pose(120, 72, Math.toRadians(90));
            follower.setStartingPose(startingpose);
            localize = new LambdaCommand()
                    .setStart(()->follower.setPose(new Pose(129,90,Math.toRadians(90))));

        }
        if(alliance ==1){
            startingpose=new Pose (24, 72, Math.toRadians(90));
            follower.setStartingPose(startingpose);
            localize = new LambdaCommand()
                    .setStart(()->follower.setPose(new Pose(15,90,Math.toRadians(90))));

        }


        startingpose = Storage.currentPose;
        follower.setStartingPose(startingpose);




        follower.update();



    }




    double goalY = 141;
    double goalX = 141;

    static double localizeX;
    double localizeY;

    double goalYDist = 141;
    double goalXDist = 141;


    static boolean shooting = false;


    public boolean isInLaunchZone(double x, double y) {

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

    public Command Localize(){
        return localize;
    }

    @Override
    public void periodic() {
        if (firsttime == true) {
            Gamepads.gamepad1().x().whenBecomesTrue((()->Localize().schedule()));
            firsttime = false;


        }
        follower.update();



        if (isBlue() == true) {
            goalXDist = 6;
            goalX = 6;
            localizeX = 136;
        }
        if (isRed() == true) {
            goalXDist = 138;
            goalX = 138;
            localizeX = 8;
        }
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
        Vector robotToGoalVector = new Vector(follower.getPose().distanceFrom(new Pose(goalX, goalY)), Math.atan2(goalY - currPose.getY(), goalX - currPose.getX()));
        //Vector v = new Vector(new Pose(138, 138));
        //ActiveOpMode.telemetry().addData("goalX", goalX);
        //ActiveOpMode.telemetry().addData("goalY", goalY);
        ActiveOpMode.telemetry().addData("RobotX", currPose.getX());
        ActiveOpMode.telemetry().addData("RobotY", currPose.getY());
        //ActiveOpMode.telemetry().addData("goalXDist", goalXDist);
        //ActiveOpMode.telemetry().addData("goalYDist", goalYDist);
        //ActiveOpMode.telemetry().addData("robotHeading", Math.toDegrees(robotHeading));
        //ActiveOpMode.telemetry().addData("velocity", follower.getVelocity());
        //ActiveOpMode.telemetry().addData("distance", distance);
        ActiveOpMode.telemetry().update();
    }
}