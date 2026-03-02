package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// ===== CHANGED: VisionPortal AprilTag imports =====
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
// ================================================
import java.util.List;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "Red Teleop")
public class CameraAutoAim extends OpMode {
    GoBildaPinpointDriver pinpoint;


    private DcMotor intakeMotor;
    private Limelight3A limelight;
    private boolean manual = false;

    private DcMotor FrontleftdrivemotorBRFL;
    private DcMotor FrontrightdrivemotorBLORF;
    private DcMotor backleftdrivemotorBLORF;
    private DcMotor backrightdrivemotorBRFL;
    private double turretzero = 0.53;

    private DcMotorEx shooter1Motor;
    private DcMotorEx shooter2Motor;
    private DcMotor transferMotor;
    private Servo counterturret;

    private Servo blocker;

    private IMU imu;
    private boolean autoshoot = true;
    private double distanceFromLimelightToGoalInches;
    private double slowModeMultiplier = 2500;
    private double currentx;
    private double currenty;
    private double targetx = 1;
    private double power;
    private double targety = 1;
    private boolean reset;
    private double driverupdown = 0;

    private boolean forward = false;
    private boolean backward = false;
    private boolean turret = true;
    private boolean transfer;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private ElapsedTime timer = new ElapsedTime();
    private boolean ingate = false;



    // Controls
    private boolean slowMode = false;
    private double shoterpower;
    private boolean intake = false;
    private double degreesmoved;
    private ElapsedTime transfertimer = new ElapsedTime();
    private boolean six = false;
    private boolean intakeforward = true;
    private boolean pidhigh = false;
    private Supplier<PathChain> pathChain;


    // PID constants for turret
    private double Kp = 0.027;
    private double turretOutput;
    private boolean close = true;
    private double atagtarget;
    private boolean pinned = false;
    private boolean far = false;

    private double t1urret;


    private double offset = 0;


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
    private double getShooterPower(double distance) {
        shoterpower=-0.00000618753*(distance*distance*distance*distance)+0.00217717*(distance*distance*distance)-0.220607*(distance*distance)+7.57294*distance+1497.90135;
        return shoterpower;
    }
    private void feildCentricDrive(double pinpointDistance) {
        double distanceToTarget = pinpointDistance;
        if (pinpointDistance < 0) {
            distanceToTarget += (2 * (Math.PI));
        }

        double y = Math.cos(distanceToTarget) * (gamepad1.right_stick_y) + (gamepad1.right_stick_x) * (Math.sin(distanceToTarget));
        double x = Math.cos(distanceToTarget) * (gamepad1.right_stick_x) - (gamepad1.right_stick_y) * (Math.sin(distanceToTarget));
        double correction = (gamepad1.left_stick_x)/6;

        ((DcMotorEx) FrontleftdrivemotorBRFL).setVelocity(((y - x) - (gamepad1.left_stick_x) - correction) * slowModeMultiplier);
        ((DcMotorEx) FrontrightdrivemotorBLORF).setVelocity(((y + x) + (gamepad1.left_stick_x) + correction) * slowModeMultiplier);
        ((DcMotorEx) backleftdrivemotorBLORF).setVelocity(((y + x) - (gamepad1.left_stick_x) - correction) * slowModeMultiplier);
        ((DcMotorEx) backrightdrivemotorBRFL).setVelocity(((y - x) + (gamepad1.left_stick_x) + correction) * slowModeMultiplier);


    }
    private void pinpointautoaim() {
        Pose2D pos2D = pinpoint.getPosition();
        double turretX = 2925 + pos2D.getX(DistanceUnit.MM) + (Math.cos(pos2D.getHeading(AngleUnit.RADIANS)));
        double turretY =(-2040 + pos2D.getY(DistanceUnit.MM) + (Math.sin(pos2D.getHeading(AngleUnit.RADIANS))));
        double newDistY = -144 + (3657.6 - turretY)/25.4;
        telemetry.addData("newy", newDistY);
        telemetry.addData("turrety", turretY);
        double distX = (3657.6 - Math.abs(turretX))/25.4;
        double fieldAngleRad = Math.atan2(newDistY, distX);
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);
        double robotHeadingDeg = pos2D.getHeading(AngleUnit.DEGREES);
        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;
        double turnneed = turretTargetDeg;
        counterturret.setPosition(turretzero + turnneed/665);
        if (counterturret.getPosition() > (turretzero + 0.16)) {
            counterturret.setPosition(turretzero + 0.16);
        }
        if (counterturret.getPosition() < (turretzero - 0.25)) {
            counterturret.setPosition(turretzero - 0.25);
        }
    }
    private boolean nogatehit() {
        Pose2D pos2D = pinpoint.getPosition();
        double turretX = 2925 + pos2D.getX(DistanceUnit.MM) + (Math.cos(pos2D.getHeading(AngleUnit.RADIANS)));
        double turretY =(-2000 + pos2D.getY(DistanceUnit.MM) + (Math.sin(pos2D.getHeading(AngleUnit.RADIANS))));
        double newDistY = -144 + (3657.6 - turretY)/25.4;
        telemetry.addData("newy", newDistY);
        double distY = (3657.6 - turretY)/25.4;
        telemetry.addData("turrety", turretY);
        double distX = (3657.6 - Math.abs(turretX))/25.4;
        double direction = Math.atan((gamepad1.right_stick_y)/(gamepad1.right_stick_x));
        telemetry.addData("direction", direction);
        telemetry.addData("x", distX);
        telemetry.addData("y", distY);
        if (distX > 121 && distX < 141 && distY > 73 && distY < 93) {
            telemetry.addData("in zone", "true");
            ingate = false;
        } else {
            ingate = true;
        }

        if (direction > 0 && direction < Math.PI/4) {
            telemetry.addData("over 0 under pi/4", "true");
        } else if (direction < 0 && direction > (2*Math.PI - Math.PI/4)) {
            telemetry.addData("under 0 over 7pi/4", "true");
        }
        /*if (131 < distY && 141 > distY && distX > 78 && distX < 88) {
            telemetry.addData("are u in the zone?", "your in the FUCKING ZONE GET THE FUCK OUT OF THERE");
        }*/
        return ingate;
    }
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;
    static final double TAG_SIZE_METERS = 0.1651;


    @Override
    public void init() {
        turretOutput = 0;
        double voltage = getBatteryVoltage();
        double KF = 50 * 10.8 / voltage;

        // initializing and configuring the pinpoint
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        // initializing drive motors
        FrontleftdrivemotorBRFL = hardwareMap.get(DcMotor.class, "Front left drive motor BRFL");
        FrontrightdrivemotorBLORF = hardwareMap.get(DcMotor.class, "Front right drive motor BLORF");
        backleftdrivemotorBLORF = hardwareMap.get(DcMotor.class, "back left drive motor BLORF");
        backrightdrivemotorBRFL = hardwareMap.get(DcMotor.class, "back right drive motor BRFL");

        FrontleftdrivemotorBRFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontrightdrivemotorBLORF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftdrivemotorBLORF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightdrivemotorBRFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        FrontleftdrivemotorBRFL.setDirection(DcMotor.Direction.FORWARD);
        backleftdrivemotorBLORF.setDirection(DcMotor.Direction.FORWARD);
        FrontrightdrivemotorBLORF.setDirection(DcMotor.Direction.REVERSE);
        backrightdrivemotorBRFL.setDirection(DcMotor.Direction.REVERSE);

        // initializing transfer motor
        intakeMotor = hardwareMap.get(DcMotor.class, "Transfer_Motor");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //initializing shooter motors
        shooter1Motor = hardwareMap.get(DcMotorEx.class, "LeftShooter_Motor");
        shooter1Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter1Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter1Motor.setVelocityPIDFCoefficients(0.2, 0, 0, KF);
        shooter2Motor = hardwareMap.get(DcMotorEx.class, "RightShooter_Motor");
        shooter2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter2Motor.setVelocityPIDFCoefficients(0.2, 0, 0, KF);

        // initializing intake motors
        transferMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // initializing servo
        counterturret = hardwareMap.servo.get("counterturret");
        blocker = hardwareMap.servo.get("blocker");


        // initializing and configuring webby (logitech c270)
        aprilTag = new AprilTagProcessor.Builder()

                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webby"))
                .addProcessor(aprilTag)
                .build();

        telemetry.setMsTransmissionInterval(10);
        // initializing and configuring interal IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    @Override
    public void start() {

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        // PID battery stuff
        /*double svoltage = getBatteryVoltage();
        double sKF = 150 / svoltage;


        // turret control
        pinpointautoaim();
        Pose2D pos2D = pinpoint.getPosition();
        double turretX = 2925 + pos2D.getX(DistanceUnit.MM) + (Math.cos(pos2D.getHeading(AngleUnit.RADIANS)));
        double turretY =(-2000 + pos2D.getY(DistanceUnit.MM) + (Math.sin(pos2D.getHeading(AngleUnit.RADIANS))));
        double y = -144 + (3657.6 - turretY)/25.4;
        telemetry.addData("newy", y);
        if (gamepad2.rightBumperWasPressed()) {
            turretzero -= 0.005;
        }
        if (gamepad2.leftBumperWasPressed()) {
            turretzero += 0.005;
        }
        double dx = (3657.6 - Math.abs(turretX))/25.4;

        double x = Math.sqrt(dx*dx+y*y);
        double power = 0.00000587054 * x*x*x*x - 0.00310024 * x*x*x + 0.62862 * x*x-52.04003 * x+3014.4987;
        telemetry.addData("distance", x);
        shooter1Motor.setVelocityPIDFCoefficients(50, 0, 0, sKF);
        shooter2Motor.setVelocityPIDFCoefficients(50, 0, 0, sKF);

        if (power > 1900) {
            power = 1900;
        }
        shooter1Motor.setVelocity(-1 * power);
        shooter2Motor.setVelocity(power);
        if (gamepad1.xWasPressed()) {
            pidhigh = !pidhigh;
        }





        // intake motor control
        transferMotor.setPower(1);

        if (gamepad1.x) {
            transferMotor.setPower(-1);
        }
        boolean nohit = nogatehit();
        if (gamepad1.yWasPressed()){
            if (slowModeMultiplier == 2500){
                slowModeMultiplier = 500;
            } else if (!nohit){
                slowModeMultiplier = 2500;

            } else if (nohit) {
                slowModeMultiplier = 500;
            }
        }

        // transfer motor control
        if (gamepad1.rightBumperWasPressed() || gamepad1.aWasPressed()) {
            transfer = !transfer;
        }
        if (gamepad1.leftBumperWasPressed()) {
            transfertimer.reset();
            six = true;
        }
        if (six = true && transfertimer.seconds() < 0.075) {
            intakeMotor.setPower(-1);
        } else if (six = true && transfertimer.seconds() > 0.075) {
            intakeMotor.setPower(0);
            six = false;
        }
        if (transfer && x < 140) {
            intakeMotor.setPower(-1);
            gamepad1.rumble(50);
            blocker.setPosition(0.65);

        } else if (transfer && x > 140){
            intakeMotor.setPower(-0.8);
            gamepad1.rumble(50);
            blocker.setPosition(0.65);
        } else {
            blocker.setPosition(0.95);

        }
        if (gamepad1.bWasPressed()) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }

        // shooter motor controls

        telemetry.addData("left", shooter1Motor.getVelocity());
        telemetry.addData("right", shooter2Motor.getVelocity());

        // fieldcentric driving*/
        Pose2D pos2D = pinpoint.getPosition();

        pinpoint.update();
        double heading = pos2D.getHeading(AngleUnit.RADIANS);
        feildCentricDrive(heading);

        List<AprilTagDetection> detections = aprilTag.getDetections();
        double turretX = 2925 + pos2D.getX(DistanceUnit.MM) + (Math.cos(pos2D.getHeading(AngleUnit.RADIANS)));
        double turretY =(-2000 + pos2D.getY(DistanceUnit.MM) + (Math.sin(pos2D.getHeading(AngleUnit.RADIANS))));
        double y = -144 + (3657.6 - turretY)/25.4;
        telemetry.addData("newy", y);
        if (gamepad2.rightBumperWasPressed()) {
            turretzero -= 0.005;
        }
        if (gamepad2.leftBumperWasPressed()) {
            turretzero += 0.005;
        }
        double dx = (3657.6 - Math.abs(turretX))/25.4;

        double x = Math.sqrt(dx*dx+y*y);

        if (!detections.isEmpty()) {
            AprilTagDetection detection = detections.get(0);

            double tagX = detection.ftcPose.x;
            double tagY = detection.ftcPose.y;
            double tagZ = detection.ftcPose.z;
            if (x > 120){
                atagtarget = 350;

            } else {
                atagtarget = 300;
            }
            double atagx = detection.center.x - 320;
            /*if  (atagx > 50) {
                counterturret.setPosition(counterturret.getPosition() - 0.000002);
            }
            if (atagx < -50) {
                 counterturret.setPosition(counterturret.getPosition() + 0.000002);
            }*/


            telemetry.addData("Tag ID", detection.id);
            telemetry.addData("Distance (in)", x);
            telemetry.addData("Distance x", detection.center.x);
            telemetry.addData("Distance y", tagY);
            telemetry.addData("Distance z", tagZ);
            telemetry.addData("PID ERROR: ", atagx);
            if (x > 140){
                atagtarget = 400;

            } else {
                atagtarget = 300;
            }


        }


        telemetry.update();
    }
    public void configurePinpoint () {
        pinpoint.setOffsets(-160.0, -50.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.resetPosAndIMU();

    }
}
