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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "far red autonomous", group = "Examples")
public class redfarauto extends OpMode {
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
     * Lets assume our robot is 16 by 16 iknches
     * Lets assume the Robot is facing the Blue Goal when it is located above the tape line and touching the Blue Goal */

    /** Start Pose of our robot */
    private final Pose start1Pose = new Pose(10, 77, Math.toRadians(0));
    private final Pose start5Pose = new Pose(10, 77, Math.toRadians(0));
    private final Pose start6Pose = new Pose(40, 110, Math.toRadians(-135));


    private final Pose start2Pose = new Pose(1.5, 74, Math.toRadians(0));
    private final Pose start4Pose = new Pose(4, 71, Math.toRadians(0));

    private final Pose start3Pose = new Pose(1.5, 59, Math.toRadians(0));
    private final Pose startPose = new Pose (-6.5, -0, Math.toRadians(0));
    private final Pose buddyPose = new Pose (5, 1, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the goal at a 135 degree angle. */
    private final Pose scorePose = new Pose(11, 30, Math.toRadians(45));
    private final Pose scoreclosePose = new Pose (15, 10, Math.toRadians(0));

    /** Highest (First Set) of Artifacts from the Spike Mark */
    private final Pose pickup1closePose = new Pose (1, 33, Math.toRadians(0));
    private final Pose pickup1Pose = new Pose(47, 33.5, Math.toRadians(0));
    private final Pose humanareaPose = new Pose(55,0, Math.toRadians(0));

    /** Middle (Second Set) of Artifacts from the Spike Mark */
    private final Pose pickup2Pose = new Pose(47, 57, Math.toRadians(0));
    private final Pose pickup2closePose = new Pose (1, 57, Math.toRadians(0));
    private final Pose gatePose = new Pose (40, 64, Math.toRadians(0));
    private final Pose endgatePose = new Pose (28, 63, Math.toRadians(0));

    private final Pose gashPose = new Pose (33, 64, Math.toRadians(0));

    private final Pose gategrabPose = new Pose (38.5, 51, Math.toRadians(35));
    private final Pose gashgrabPose = new Pose (28.5, 51, Math.toRadians(35));
    /** Lowest (Third Set) of Artifacts from the Spike Mark */
    private final Pose pickup3Pose = new Pose(35, 70, Math.toRadians(0));
    private final Pose pickup3closePose = new Pose (1, 70, Math.toRadians(0));
    private final Pose humanclosePose = new Pose (51, 6, Math.toRadians(-90));
    private final Pose humanPose = new Pose (51, 0, Math.toRadians(-90));



    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload;
    private PathChain pushem, antipushem, scoreclosePickup1,score2,score3, grabPickup1,scoreHuman, grabHumanclose, grabHuman, grabHumansigma, moveoffline, moveBuddy, superscore, pashGate, scoreclosePickup2, grabsigmaPickup1, grabPickup2, grabsigmaPickup2, pushGate, grabPickup3, grabsigmaPickup3, scorePickup1, scorePickup2, scorePickup3;
    private DcMotor intakeMotor;
    private double distY = 0;
    private double turretOutput;
    private double t1urret;
    private Servo blocker;
    private boolean shouldshoot;
    private boolean shouldshat;
    private DcMotorEx shooter1Motor;
    private DcMotorEx shooter2Motor;
    private boolean intake = false;
    private DcMotor transferMotor;
    private Servo counterturret;
    private IMU imu;

    private double Kp = 0.002;
    private double error;
    private double shooter = 1800;
    private double velo = 0;
    private double out;
    private ElapsedTime transferTimer = new ElapsedTime();
    private ElapsedTime intakeTimer = new ElapsedTime();

    private double robotx;
    private double roboty;
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
        grabsigmaPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, pickup1closePose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), pickup1closePose.getHeading())
                .build();
        /*pushem = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pushpose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pushpose.getHeading())
                .build();
        antipushem = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, startPose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), startPose.getHeading())
                .build();*/

        score2 = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, pickup2closePose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), pickup1closePose.getHeading())
                .build();
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, start4Pose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), start4Pose.getHeading())
                .build();

        moveBuddy = follower.pathBuilder()
                .addPath(new BezierLine(buddyPose, start2Pose))
                .setLinearHeadingInterpolation(buddyPose.getHeading(), start2Pose.getHeading())
                .build();
        moveoffline = follower.pathBuilder()
                .addPath(new BezierLine(startPose, buddyPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), buddyPose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1closePose, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1closePose.getHeading(), pickup1Pose.getHeading())
                .build();
        scoreclosePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, pickup1closePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), pickup1closePose.getHeading())
                .build();
        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, start5Pose))
                .setLinearHeadingInterpolation(pickup1closePose.getHeading(), startPose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pushGate = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, gashPose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), gashPose.getHeading())
                .build();
        pashGate = follower.pathBuilder()
                .addPath(new BezierLine(gashPose, gatePose))
                .setLinearHeadingInterpolation(gashPose.getHeading(), gatePose.getHeading())
                .build();



        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreclosePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2closePose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2closePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(gashPose, start4Pose))
                .setLinearHeadingInterpolation(gashPose.getHeading(), startPose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabsigmaPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup3closePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup3closePose.getHeading())
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3closePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, start1Pose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), startPose.getHeading())
                .build();
        superscore = follower.pathBuilder()
                .addPath(new BezierLine(start1Pose, endgatePose))
                .setLinearHeadingInterpolation(start1Pose.getHeading(), gatePose.getHeading())
                .build();
        grabHuman = follower.pathBuilder()
                .addPath(new BezierLine(humanclosePose, humanPose))
                .setLinearHeadingInterpolation(humanclosePose.getHeading(), humanPose.getHeading())
                .build();
        grabsigmaPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, pickup2closePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), pickup2closePose.getHeading())
                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2closePose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2closePose.getHeading(), pickup2Pose.getHeading())

                .build();

        grabHumansigma = follower.pathBuilder()
                .addPath(new BezierLine(start2Pose, humanclosePose))
                .setLinearHeadingInterpolation(start2Pose.getHeading(), humanclosePose.getHeading())
                .build();
        scoreHuman = follower.pathBuilder()
                .addPath(new BezierLine(humanPose, start1Pose))
                .setLinearHeadingInterpolation(humanPose.getHeading(), start2Pose.getHeading())
                .build();
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                intake = false;
                transferMotor.setPower(0);
                follower.followPath(moveoffline, 0.9, true);
                setPathState(1);
                break;
            case 1:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(100);
                }
            case 100:
                intake = false;
                transferMotor.setPower(0);
                follower.followPath(moveBuddy, 1, true);
                setPathState(200);
                velo = 1555;
                intakeTimer.reset();
                break;
            case 200:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(2);
                }
            case 2:
                shooter = 1600;
                intake = true;
                if (!follower.isBusy()) {
                    if (shooter2Motor.getVelocity() > 1500 && transferTimer.seconds() < 1 && transferTimer.seconds() > 0.25) {
                        shouldshoot = true;
                        velo = 1555;
                    } else if (transferTimer.seconds() > 1){
                        setPathState(16);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }

                break;
            case 3:
                if (!follower.isBusy()) {
                    intake = false;
                    follower.followPath(score2, 1, true);
                    setPathState(4);
                }
                break;
            case 28:
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            case 4:
                if(!follower.isBusy()) {
                    intake = true;
                    follower.followPath(grabPickup2,0.9, true);
                    setPathState(3000);
                    if (follower.getCurrentTValue() > 0.45 && follower.getCurrentTValue() < 0.850) {
                        transferMotor.setPower(0.5);
                    } else {
                        transferMotor.setPower(0);
                    }
                }
                break;
            case 10:
                if(!follower.isBusy()) {

                    transferMotor.setPower(0);
                    follower.followPath(grabsigmaPickup1, 1, true);
                    setPathState(11);
                }
                break;
            case 11:
                if(!follower.isBusy()) {
                    intake = true;
                    follower.followPath(grabPickup1,0.9,true);
                    if (follower.getCurrentTValue() > 0.40 && follower.getCurrentTValue() < 0.80) {
                        transferMotor.setPower(1);
                    } else {
                        transferMotor.setPower(0);
                    }
                    setPathState(12);

                }
                break;
            case 2020:
                if(!follower.isBusy()) {
                    intake = true;
                    follower.followPath(scoreclosePickup1, 1,true);
                    setPathState(12);
                }
                break;
            case 12:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup1, 1,true);
                    setPathState(13);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(14);
                }
            case 14:
                intake = true;
                shooter = 1625;
                intake = true;
                if (!follower.isBusy()) {
                    if (shooter2Motor.getVelocity() > 1500 && transferTimer.seconds() < 1 && transferTimer.seconds() > 0.25) {
                        shouldshoot = true;
                    } else if (transferTimer.seconds() > 1){
                        setPathState(22);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;
            case 7:
                if(!follower.isBusy()) {
                    follower.followPath(scoreclosePickup2, 1,true);
                    setPathState(7);
                }
                break;
            case 6:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup2, 1,true);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(9);
                }
                break;
            case 9:
                intake = true;
                shooter = 1600;
                intake = true;
                if (!follower.isBusy()) {
                    if (shooter2Motor.getVelocity() > 1500 && transferTimer.seconds() < 1 && transferTimer.seconds() > 0.25) {
                        shouldshoot = true;
                    } else if (transferTimer.seconds() > 1){
                        setPathState(10);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;
            case 15:
                if(!follower.isBusy()) {
                    follower.followPath(score3, 1, true);
                    setPathState(95);

                }
                break;
            case 95:
                if(!follower.isBusy()) {
                    transferMotor.setPower(0);
                    follower.followPath(grabsigmaPickup3, 1,true);
                    setPathState(96);
                }
                break;
            case 96:
                if(!follower.isBusy()) {
                    setPathState(16);
                }
                break;
            case 16:
                if(!follower.isBusy()) {
                    intake = true;
                    follower.followPath(grabPickup3, 0.9,true);
                    if (follower.getCurrentTValue() > 0.45 && follower.getCurrentTValue() < 0.80) {
                        transferMotor.setPower(1);
                    } else {
                        transferMotor.setPower(0);
                    }
                    setPathState(17);
                }
                break;
            case 3000:
                if(!follower.isBusy()) {
                    follower.followPath(pushGate, 1, true);
                    setPathState(3001);
                }
                break;
            case (3001):
                if (!follower.isBusy()) {
                    follower.followPath(pashGate, 1, true);
                    setPathState(3002);
                }break;
            case (3002):
                if (!follower.isBusy()) {
                    transferTimer.reset();
                }
                setPathState(3003);
                break;
            case (3003):
                if (!follower.isBusy()) {
                    if (transferTimer.seconds() > 0.2) {
                        setPathState(6);
                    }
                }
                break;
            case 17:
                if(!follower.isBusy()) {
                    follower.followPath(scorePickup3, 1,true);
                    setPathState(18);
                }
                break;
            case 18:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(23);
                }
                break;
            case 23:
                intake = true;
                if (!follower.isBusy()) {
                    if (shooter2Motor.getVelocity() > 1500 && transferTimer.seconds() < 1 && transferTimer.seconds() > 0.25) {
                        shouldshoot = true;
                        transferMotor.setPower(1);
                    } else if (transferTimer.seconds() > 1){
                        setPathState(3);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }


                break;

            /*case 22:
                if(!follower.isBusy()) {
                    transferMotor.setPower(0);
                    follower.followPath(pushem, 1,true);
                    setPathState(100);
                }
                break;
            case 100:
                if(!follower.isBusy()) {
                    follower.followPath(superscore, 1, true);
                    setPathState(21);
                }
                break;
            case 25:
                if(!follower.isBusy()) {
                    follower.followPath(antipushem, 0.7, false);
                    setPathState(22);
                }*/
            case 22:
                if(!follower.isBusy()) {
                    intake = true;
                    transferMotor.setPower(0);
                    follower.followPath(grabHumansigma, 0.95,true);
                    setPathState(4000);
                }
                break;
            /*case 24:
                if(!follower.isBusy()) {
                    transferMotor.setPower(0);
                    follower.followPath(grabHuman, 1,false);
                    setPathState(4000);
                }
                break;*/
            case 4000:
                if(!follower.isBusy()) {
                    transferMotor.setPower(0);
                    follower.followPath(scoreHuman, 1,true);
                    setPathState(30);
                }
                break;
            case 30:
                if (!follower.isBusy()) {
                    transferTimer.reset();
                    setPathState(31);
                }
                break;
            case 31:
                intake = true;
                if (!follower.isBusy()) {
                    if (shooter2Motor.getVelocity() > 1500 && transferTimer.seconds() < 2 && transferTimer.seconds() > 1) {
                        shouldshoot = true;
                    } else if (transferTimer.seconds() > 1.75){
                        setPathState(2100);
                        transferMotor.setPower(0);
                        shouldshoot = false;
                    }
                }
                break;

            case 2100:
                if (!follower.isBusy()) {
                    follower.followPath(superscore, 0.7, true);
                    setPathState(21);
                }
                break;
            case 21:
                if(!follower.isBusy()) {
                    setPathState(-1);
                }
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
        if (result > 13.3) {
            result = 13.3;
        }
        return result;
    }
    private void pinpointautoaim() {
        double heading = Math.toDegrees(Math.PI - follower.getPose().getHeading());
        telemetry.addData("heading", heading);
        double turretX = (86 + follower.getPose().getX()) + (Math.cos(follower.getPose().getHeading()));
        double turretY = (-follower.getPose().getY()) + (Math.sin(follower.getPose().getHeading()));
        if (
                pathState == 3  || pathState == 28 || pathState == 4  ||
                        pathState == 6  || pathState == 8  || pathState == 9  ||
                        pathState == 10 || pathState == 11 || pathState == 2020 ||
                        pathState == 12 || pathState == 13 || pathState == 14 ||
                        pathState == 22 || pathState == 4000 ||
                        pathState == 30 || pathState == 31 || pathState == 21
        ) {
            distY = (136 - Math.abs(turretY));
        } else {
            distY = (136 - Math.abs(turretY));

        }
        double distX = (144 - Math.abs(turretX));
        double fieldAngleRad = Math.atan2(distY, distX);
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);
        double robotHeadingDeg = Math.toDegrees(follower.getPose().getHeading());
        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;
        double turnneed = turretTargetDeg;
        if (pathState  == -2) {
            counterturret.setPosition(0.519 + turnneed/665);
        } else if (pathState > 24){
            counterturret.setPosition(0.519 + turnneed / 665);
        } else {
            counterturret.setPosition(0.519 + turnneed / 665);
        }
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


        telemetry.addData("servo pos", counterturret.getPosition());

        telemetry.addData("path: ", pathState);
        telemetry.update();
        pinpointautoaim();
        robotx = follower.getPose().getX() + (Math.cos(follower.getPose().getHeading()));

        roboty = -follower.getPose().getY() + (Math.sin(follower.getPose().getHeading()));
        double y = (144 - Math.abs(robotx));
        double dx = (144 - Math.abs(roboty));
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
            shooter2Motor.setVelocity(1540);
            shooter1Motor.setVelocity(-1540);
        } else if (pathState <= 16){
            shooter2Motor.setVelocity(1540);
            shooter1Motor.setVelocity(-1540);
        } else  if (pathState == 30) {
            shooter2Motor.setVelocity(1540);
            shooter1Motor.setVelocity(-1540);
        } else if ( pathState == 22){
            shooter2Motor.setVelocity(1540);
            shooter1Motor.setVelocity(-1540);
        } else if (pathState == 41){
            shooter2Motor.setVelocity(1540);
            shooter1Motor.setVelocity(-1540);
        } else  if (pathState == 3000){
            shooter2Motor.setVelocity(1540);
            shooter1Motor.setVelocity(-1540);
        }else {
            shooter2Motor.setVelocity(1540);
            shooter1Motor.setVelocity(-1540);
        }
        if (shouldshoot) {
            transferMotor.setPower(1);
            blocker.setPosition(0.95);
        } else {
            transferMotor.setPower(0);
            blocker.setPosition(0.75);
        }
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
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

        blocker = hardwareMap.servo.get("blocker");

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
}