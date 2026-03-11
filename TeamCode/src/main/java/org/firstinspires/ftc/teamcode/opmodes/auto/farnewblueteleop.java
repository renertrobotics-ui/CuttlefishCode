package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "RED buddy Auto", group = "Examples")
public class farnewblueteleop extends OpMode {
    GoBildaPinpointDriver pinpoint;
    private Follower follower;
    private Timer pathTimer, opmodeTimer;




    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Decode, this would be Red Loading Zone (0,0) to Red Goal (144,144).)
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://visualizer.pedropathing.com/>
     * Lets assume our robot is 16 by 16 inches
     * Lets assume the Robot is facing the Blue Goal when it is located above the tape line and touching the Blue Goal */

    /** Start Pose of our robot */
    private final Pose start1Pose = new Pose(10, 77, Math.toRadians(0));
    private final Pose start5Pose = new Pose(15, 80, Math.toRadians(0));
    private final Pose start6Pose = new Pose(40, 110, Math.toRadians(135));


    private final Pose start2Pose = new Pose(8, 4, Math.toRadians(0));
    private final Pose start4Pose = new Pose(4, 73, Math.toRadians(0));

    private final Pose start3Pose = new Pose(6, 4, Math.toRadians(0));
    private final Pose startPose = new Pose (0, 0, Math.toRadians(0));
    private final Pose endPose = new Pose (28, 63, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the goal at a 135 degree angle. */
    private final Pose scorePose = new Pose(11, 30, Math.toRadians(-45));
    private final Pose scoreclosePose = new Pose (15, 10, Math.toRadians(0));

    /** Highest (First Set) of Artifacts from the Spike Mark */
    private final Pose pickup1closePose = new Pose (1, 19, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(49, 19, Math.toRadians(0));
    private final Pose humanareaPose = new Pose(55,0, Math.toRadians(0));

    /** Middle (Second Set) of Artifacts from the Spike Mark */
    private final Pose pickup2Pose = new Pose(46, 58, Math.toRadians(0));
    private final Pose pickup2closePose = new Pose (1, 57, Math.toRadians(0));
    private final Pose gatePose = new Pose (36.2, 68.5, Math.toRadians(0));
    private final Pose endgatePose = new Pose (40, 0, Math.toRadians(0));

    private final Pose gashPose = new Pose (31, 68.5, Math.toRadians(0));

    private final Pose gategrabPose = new Pose (38.5, 51, Math.toRadians(-35));
    private final Pose gashgrabPose = new Pose (28.5, 51, Math.toRadians(-35));
    /** Lowest (Third Set) of Artifacts from the Spike Mark */
    private final Pose pickup3Pose = new Pose(34, 71, Math.toRadians(0));
    private final Pose pickup3closePose = new Pose (1, 71, Math.toRadians(0));
    private final Pose humanclosePose = new Pose (47, 15, Math.toRadians(90));
    private final Pose humanPose = new Pose (47, 7, Math.toRadians(90));
    private final Pose humanclosePose2 = new Pose (36, 27, Math.toRadians(90));
    private final Pose humanPose2 = new Pose (49, 27, Math.toRadians(90));
    private final Pose humanclosePose3 = new Pose (36, -2, Math.toRadians(0));
    private final Pose humanPose3 = new Pose (49, -2, Math.toRadians(0));




    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain grabHumansigma, moveEnd, grabHumansigma2,grabHumansigma3, moveoffline, grabHuman, scoreHuman, grabHuman2, scoreHuman2, grabHuman3, scoreHuman3, grabsigmaPickup1, grabPickup1, scorePickup1, grabsigmaPickup2, grabPickup2, gashPush, gatePush, scoreGate, grabsigmaPickupG, grabPickupG;
    private DcMotor intakeMotor;
    private double turretOutput;
    private double t1urret;
    private boolean shouldshoot;
    private boolean shouldshat;
    private DcMotorEx shooter1Motor;
    private DcMotorEx shooter2Motor;
    private DcMotor transferMotor;
    private Servo counterturret;
    private IMU imu;

    private double Kp = 0.002;
    private double error;
    private double shooter = 1870;
    private double out;
    private double distY = 0;
    private boolean intake = false;
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime skibidtimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();
    private double sKF = 0;





    public void createPoses() {

    }


    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {
        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(start2Pose, pickup2closePose));
        scorePreload.setLinearHeadingInterpolation(start2Pose.getHeading(), pickup2closePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        moveoffline = follower.pathBuilder()
                .addPath(new BezierLine(startPose, start3Pose))
                .setLinearHeadingInterpolation(startPose.getHeading(), start3Pose.getHeading())
                .build();
        grabHumansigma = follower.pathBuilder()
                .addPath(new BezierLine(start3Pose, humanclosePose))
                .setLinearHeadingInterpolation(start3Pose.getHeading(), humanclosePose.getHeading())
                .build();
        grabHuman = follower.pathBuilder()
                .addPath(new BezierLine(humanclosePose, humanPose))
                .setLinearHeadingInterpolation(humanclosePose.getHeading(), humanPose.getHeading())
                .build();
        scoreHuman = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, start2Pose))
                .setLinearHeadingInterpolation(humanPose.getHeading(), start2Pose.getHeading())
                .build();
        grabHumansigma2 = follower.pathBuilder()
                .addPath(new BezierLine(start3Pose, humanPose2))
                .setLinearHeadingInterpolation(start3Pose.getHeading(), humanclosePose2.getHeading())
                .build();
        grabHuman2 = follower.pathBuilder()
                .addPath(new BezierLine(humanclosePose2, humanPose2))
                .setLinearHeadingInterpolation(humanclosePose2.getHeading(), humanPose2.getHeading())
                .build();
        scoreHuman2 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose2, start2Pose))
                .setLinearHeadingInterpolation(humanPose2.getHeading(), start2Pose.getHeading())
                .build();

        grabHumansigma3 = follower.pathBuilder()
                .addPath(new BezierLine(start3Pose, humanPose3))
                .setLinearHeadingInterpolation(start3Pose.getHeading(), humanPose3.getHeading())
                .build();
        grabHuman3 = follower.pathBuilder()
                .addPath(new BezierLine(humanclosePose3, humanPose3))
                .setLinearHeadingInterpolation(humanclosePose2.getHeading(), humanPose2.getHeading())
                .build();
        scoreHuman3 = follower.pathBuilder()
                .addPath(new BezierLine(humanPose3, start2Pose))
                .setLinearHeadingInterpolation(humanPose3.getHeading(), start2Pose.getHeading())
                .build();


        grabsigmaPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, pickup1closePose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), pickup1closePose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1closePose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1closePose.getHeading(), pickup1Pose.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, start2Pose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), startPose.getHeading())
                .build();
        grabsigmaPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, pickup2closePose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), pickup2closePose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2closePose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2closePose.getHeading(), pickup2Pose.getHeading())
                .build();
        gashPush = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, gashPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), gashPose.getHeading())
                .build();
        gatePush = follower.pathBuilder()
                .addPath(new BezierLine(gashPose, gatePose))
                .setLinearHeadingInterpolation(gashPose.getHeading(), gatePose.getHeading())
                .build();
        scoreGate = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, start2Pose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), start2Pose.getHeading())
                .build();
        grabsigmaPickupG = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, gashgrabPose))
                .setLinearHeadingInterpolation(gatePose.getHeading(), start2Pose.getHeading())
                .build();
        grabPickupG = follower.pathBuilder()
                .addPath(new BezierLine(gashgrabPose, gategrabPose))
                .setLinearHeadingInterpolation(gashgrabPose.getHeading(), gategrabPose.getHeading())
                .build();
        moveEnd = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endgatePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endgatePose.getHeading())
                .build();

    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                intakeMotor.setPower(0);
                transferMotor.setPower(0);
                follower.followPath(moveoffline, 0.7, true);
                skibidtimer.reset();
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy() && (shooter1Motor.getVelocity() < -1750)) {
                    transferTimer.reset();
                    setPathState(2);
                }
                break;
            case 2:
                shooter = 1700;
                intakeMotor.setPower(1);
                /*if (!follower.isBusy() && transferTimer.seconds() < 0.3) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                } else if (transferTimer.seconds() < 1) {
                    shouldshoot = false;
                    transferMotor.setPower(0);
                } else if (transferTimer.seconds() < 1.3) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                } else if (transferTimer.seconds() < 2) {
                    shouldshoot = false;
                    transferMotor.setPower(-0.4);
                    */
                if (transferTimer.seconds() < 1.25) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                    intakeMotor.setPower(1);
                } else {
                    setPathState(3);
                    transferMotor.setPower(0);
                    shouldshoot = false;
                }
                break;
            case 3:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabsigmaPickup1, 1, true);
                setPathState(4);
                break;
            case 4:
                if (!follower.isBusy()) {
                    intakeMotor.setPower(1);
                    transferTimer.reset();
                    setPathState(5);
                }
                break;
            case 5:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabPickup1, 1, false);
                setPathState(6);
                break;
            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;
            case 7:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(scorePickup1, 0.85, true);
                setPathState(8);
                break;
            case 8:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(9);
                }
                break;
            case 9:
                shooter = 1700;
                intakeMotor.setPower(1);
                if (!follower.isBusy() && transferTimer.seconds() > 0.75) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                    if (transferTimer.seconds() > 1.25){
                        setPathState(24);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;/*
            case 10:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHumansigma, 1, true);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(12);
                }
                break;
            case 12:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHuman, 1, false);
                setPathState(13);
                break;
            case 13:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(14);
                }
                break;
            case 14:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(scoreHuman, 1, true);
                setPathState(15);
                break;
            case 15:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(16);
                }
                break;
            case 16:
                shooter = 1700;
                intakeMotor.setPower(1);
                if (!follower.isBusy() && transferTimer.seconds() > 0.3) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                    if (transferTimer.seconds() > 1.5){
                        setPathState(17);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;
            case 17:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHumansigma2, 1, false);
                setPathState(18);
                break;
            case 18:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(19);
                }
                break;
            case 19:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHuman2, 1, false);
                setPathState(20);
                break;
            case 20:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(21);
                }
                break;
            case 21:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(scoreHuman2, 1, true);
                setPathState(22);
                break;
            case 22:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(23);
                }
                break;
            case 23:
                shooter = 1700;
                intakeMotor.setPower(1);
                if (!follower.isBusy() && transferTimer.seconds() > 0.3) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                    if (transferTimer.seconds() > 1.5){
                        setPathState(24);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;*/
            case 24:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHumansigma3, 1, false);
                setPathState(27);
                break;
            case 25:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(26);
                }
                break;
            case 26:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHuman3, 1, false);
                setPathState(27);
                break;
            case 27:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(28);
                }
                break;
            case 28:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(scoreHuman3, 1, true);
                setPathState(29);
                break;
            case 29:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(30);
                }
                break;
            case 30:
                shooter = 1700;
                intakeMotor.setPower(1);
                if (!follower.isBusy() && transferTimer.seconds() > 1) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                    if (transferTimer.seconds() > 1.5){
                        setPathState(31);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;
            case 31:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHumansigma2, 1, false);
                setPathState(3400);
                break;
            case 32:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(33);
                }
                break;
            case 33:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(grabHuman2, 1,false);
                setPathState(3400);
                break;
            case 3400:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(34);
                }
                break;
            case 34:
                if (transferTimer.seconds() < 0.5 && transferTimer.seconds() > 0.2) {
                    transferMotor.setPower(0.5);
                }
                if (transferTimer.seconds() > 1) {
                    setPathState(35);
                    transferMotor.setPower(0);
                }
                break;
            case 35:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(scoreHuman2, 1, true);
                setPathState(36);
                break;
            case 36:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(37);
                }
                break;
            case 37:
                shooter = 1700;
                intakeMotor.setPower(1);
                if (!follower.isBusy() && transferTimer.seconds() > 1) {
                    shouldshoot = true;
                    transferMotor.setPower(1);
                    if (transferTimer.seconds() > 1.5){
                        setPathState(31);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;




            case 50:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(51);
                }
                break;
            case 51:
                intakeMotor.setPower(1);
                transferMotor.setPower(0);
                follower.followPath(moveEnd, 0.75, true);
                setPathState(-1);
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    private double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
    private void pinpointautoaim() {
        double heading = Math.toDegrees(Math.PI - follower.getPose().getHeading());
        telemetry.addData("heading", heading);
        double turretX = (86 + follower.getPose().getX()) + (Math.cos(follower.getPose().getHeading()));
        double turretY = (-follower.getPose().getY()) + (Math.sin(follower.getPose().getHeading()));
        distY = (136 - Math.abs(turretY));


        double distX = (144 - Math.abs(turretX));
        double fieldAngleRad = Math.atan2(distY, distX);
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;
        double turnneed = turretTargetDeg;
        counterturret.setPosition(0.5 + turnneed / 665);

    }



    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        turretOutput = 0;
        double svoltage = getBatteryVoltage();
        intake = true;
        if (svoltage < 13.2) {
            sKF = 15.5 * 10.85 / svoltage;

        } else {
            sKF = 15.5*10.85 / 13.2;
        }
        shooter2Motor.setVelocityPIDFCoefficients(50, 0, 0, sKF);
        shooter1Motor.setVelocityPIDFCoefficients(50, 0, 0, sKF);

        follower.update();
        autonomousPathUpdate();

        if (skibidtimer.seconds() > 26 && skibidtimer.seconds() < 28) {
            setPathState(50);
            telemetry.addData("hi", "hi");
        }


        telemetry.addData("servo pos", counterturret.getPosition());

        telemetry.addData("path", pathState);
        telemetry.addData("vel2", shooter2Motor.getVelocity());
        telemetry.addData("vel1", shooter1Motor.getVelocity());
        telemetry.update();
        pinpointautoaim();
        double robotx = follower.getPose().getX() + (Math.cos(follower.getPose().getHeading()));

        double roboty = -follower.getPose().getY() + (Math.sin(follower.getPose().getHeading()));
        double y = (141 - Math.abs(robotx));
        double dx = (141 - Math.abs(roboty));
        telemetry.addData("Odo X", robotx);
        telemetry.addData("Odo Y", roboty);
        if (intake) {
            if (intakeTimer.seconds() > 0.05) {
                intakeTimer.reset();
            } else {
                intakeMotor.setPower(1);
            }
        } else {
            intakeMotor.setPower(0);
        }


        double x = Math.sqrt(dx*dx+y*y);
        double power = 0.00000587054 * x*x*x*x - 0.00310024 * x*x*x + 0.62862 * x*x-52.04003 * x+3009.4987;
        telemetry.addData("distance", x);

        if (pathState <= 2) {
            shooter2Motor.setVelocity(1875);
            shooter1Motor.setVelocity(-1875);
        } else if (pathState <= 16){
            shooter2Motor.setVelocity(1875);
            shooter1Motor.setVelocity(-1875);
        } else  if (pathState == 30) {
            shooter2Motor.setVelocity(1875);
            shooter1Motor.setVelocity(-1875);
        } else if ( pathState == 22){
            shooter2Motor.setVelocity(1875);
            shooter1Motor.setVelocity(-1875);
        } else if (pathState == 41){
            shooter2Motor.setVelocity(1875);
            shooter1Motor.setVelocity(-1875);
        } else  if (pathState == 3000){
            shooter2Motor.setVelocity(1875);
            shooter1Motor.setVelocity(-1875);
        }else {
            shooter2Motor.setVelocity(1875);
            shooter1Motor.setVelocity(-1875);
        }
        if (shouldshoot) {
            transferMotor.setPower(1);
        } else {
            transferMotor.setPower(0);
        }
    }

    public void init() {
        pathTimer = new Timer();
        double voltage = getBatteryVoltage();
        double KF = 15.5 * 10.85 / voltage;

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shooter1Motor = hardwareMap.get(DcMotorEx.class, "LeftShooter_Motor");
        shooter1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1Motor.setVelocityPIDFCoefficients(0.2, 0, 0, KF);
        shooter2Motor = hardwareMap.get(DcMotorEx.class, "RightShooter_Motor");
        shooter2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2Motor.setVelocityPIDFCoefficients(0.2, 0, 0, KF);
        intakeMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        transferMotor = hardwareMap.get(DcMotor.class, "Transfer_Motor");
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.setMsTransmissionInterval(11);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        counterturret = hardwareMap.servo.get("counterturret");


        follower = Constants.createFollower(hardwareMap);
        createPoses();
        buildPaths();
        follower.setStartingPose(startPose);


    }
    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        transferTimer.reset();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
    public void configurePinpoint () {
        pinpoint.setOffsets(-160.0, -50.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

    }
}