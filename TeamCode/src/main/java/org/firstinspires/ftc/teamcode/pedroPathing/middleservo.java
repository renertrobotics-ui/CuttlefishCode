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
@TeleOp(name = "servo move to middle 4 autoaim")
public class middleservo extends OpMode {
    GoBildaPinpointDriver pinpoint;


    private DcMotor intakeMotor;
    private Limelight3A limelight;
    private boolean manual = false;

    private DcMotor FrontleftdrivemotorBRFL;
    private DcMotor FrontrightdrivemotorBLORF;
    private DcMotor backleftdrivemotorBLORF;
    private DcMotor backrightdrivemotorBRFL;

    private DcMotorEx shooter1Motor;
    private DcMotorEx shooter2Motor;
    private DcMotor transferMotor;
    private Servo counterturret;

    private Servo clockwiseturret;

    private IMU imu;
    private boolean autoshoot = true;
    private double distanceFromLimelightToGoalInches;
    private double currentx;
    private double currenty;
    private double targetx = 1;
    private double power;
    double x = 0.513;

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



    // Controls
    private boolean slowMode = false;
    private double shoterpower;
    private boolean intake = false;
    private double degreesmoved;
    private boolean intakeforward = true;
    private Supplier<PathChain> pathChain;


    // PID constants for turret
    private double Kp = 0.027;
    private double turretOutput;
    private boolean close = true;
    private boolean pinned = false;
    private boolean far = false;

    private double t1urret;


    private double slowModeMultiplier = 0.25;
    private double offset = 0;

    private void autoaim() {
        if (!manual) {
            autoautoaim();
        }
        else if (manual) {
            far = Far();
            if (far) {
                if (gamepad2.rightBumperWasPressed()) {
                    counterturret.setPosition(counterturret.getPosition() + 0.0010);
                } else if (gamepad2.leftBumperWasPressed()) {
                    counterturret.setPosition(counterturret.getPosition() - 0.001);
                }
            }
            if (!far) {
                if (gamepad2.rightBumperWasPressed()) {
                    counterturret.setPosition(counterturret.getPosition() + 0.0020);
                } else if (gamepad2.leftBumperWasPressed()) {
                    counterturret.setPosition(counterturret.getPosition() - 0.002);
                }
            }
        }
    }
    private double GetDistance() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            double angleToGoalRadians = (10 + llResult.getTy()) * 3.14159 / 180;
            distanceFromLimelightToGoalInches = 33 / Math.tan(angleToGoalRadians);
        } else {
            Pose2D pos2D = pinpoint.getPosition();
            double robotx = pos2D.getX(DistanceUnit.MM) + (Math.cos(pos2D.getHeading(AngleUnit.RADIANS)));
            double roboty = pos2D.getY(DistanceUnit.MM) + (Math.sin(pos2D.getHeading(AngleUnit.RADIANS)));

            distanceFromLimelightToGoalInches = Math.sqrt(robotx*robotx+roboty*roboty);
        }
        return distanceFromLimelightToGoalInches;
    }
    private boolean Far() {
        double distance = GetDistance();
        if (distance > 200) {
            return true;
        } else {
            return false;
        }
    }
    private void autoautoaim() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            if (far) {
                t1urret = llResult.getTx() - 5;
            } else {
                t1urret = llResult.getTx();
            }
            if ((t1urret <= -3.5) && (clockwiseturret.getPosition() > 0.0025) && (counterturret.getPosition() < 0.9975)) {
                counterturret.setPosition(counterturret.getPosition() + 0.0020);
            } else if ((t1urret >= 3.5) && (clockwiseturret.getPosition() < 0.9975) && (counterturret.getPosition() > 0.0025)) {
                counterturret.setPosition(counterturret.getPosition() - 0.0020);
            }
        } else {
            distanceFromLimelightToGoalInches = 140;
            pinpointautoaim();
        }
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
    private double getShooterPower(double distance) {
        shoterpower=-0.0000112896*(distance*distance*distance*distance)+0.00664021*(distance*distance*distance)-1.4102*(distance*distance)+131.0725*distance-2959;
        return shoterpower;
    }
    private void feildCentricDrive(double pinpointDistance) {
        double distanceToTarget = pinpointDistance;
        if (pinpointDistance < 0) {
            distanceToTarget += (2 * (Math.PI));
        }

        double y = Math.cos(distanceToTarget) * (gamepad1.right_stick_y) + (gamepad1.right_stick_x) * (Math.sin(distanceToTarget));
        double x = Math.cos(distanceToTarget) * (gamepad1.right_stick_x) - (gamepad1.right_stick_y) * (Math.sin(distanceToTarget));
        double correction = -(gamepad1.left_stick_x)/4;

        ((DcMotorEx) FrontleftdrivemotorBRFL).setVelocity(((y - x) - (-gamepad1.left_stick_x) - correction) * 2000);
        ((DcMotorEx) FrontrightdrivemotorBLORF).setVelocity(((y + x) + (-gamepad1.left_stick_x) + correction) * 2000);
        ((DcMotorEx) backleftdrivemotorBLORF).setVelocity(((y + x) - (-gamepad1.left_stick_x) - correction) * 2000);
        ((DcMotorEx) backrightdrivemotorBRFL).setVelocity(((y - x) + (-gamepad1.left_stick_x) + correction) * 2000);


    }
    private void pinpointautoaim() {
        Pose2D pos2D = pinpoint.getPosition();
        double turretX = pos2D.getX(DistanceUnit.MM) + (Math.cos(pos2D.getHeading(AngleUnit.RADIANS)));
        double turretY = pos2D.getY(DistanceUnit.MM) + (Math.sin(pos2D.getHeading(AngleUnit.RADIANS)));
        double distY = (3657.6 - Math.abs(turretY))/25.4;
        double distX = (3657.6 - Math.abs(turretX))/25.4;
        double fieldAngleRad = Math.atan2(distY, distX);
        double fieldAngleDeg = Math.toDegrees(fieldAngleRad);
        double robotHeadingDeg = -pos2D.getHeading(AngleUnit.DEGREES);
        double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;
        double turnneed = turretTargetDeg;
        counterturret.setPosition(0.5-turnneed/750);
    }

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

        // initializing transfer motors
        transferMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // initializing servo
        counterturret = hardwareMap.servo.get("counterturret");

        // initializing and configuring limelight 3a
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(0);
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
        counterturret.setPosition(0.5);
    }

    @Override
    public void loop() {
        if (gamepad1.xWasPressed()) {
            x -= 0.05;
        }
        if (gamepad1.yWasPressed()) {
            x -= 0.01; }
        if (gamepad1.bWasPressed()){
            x += 0.02;
        }
        telemetry.addData("x", x);
        counterturret.setPosition(x);
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
