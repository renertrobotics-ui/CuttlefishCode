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
    @TeleOp(name = "TuneDaServos")
    public class PIDTUNING extends OpMode {
        GoBildaPinpointDriver pinpoint;

        private ElapsedTime transferTimer = new ElapsedTime();

        private DcMotor intakeMotor;

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
        private double targety = 1;
        private boolean reset;
        private double driverupdown = 0;
        private double p = 0.2;
        private double d = 0.5;
        private double i = 0.4;
        private double tt = 0.2;
        private double s = 1600;
        private double f = 5;


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
        private boolean far_zone_on = false;
        private boolean intake = false;
        private double degreesmoved;
        private Supplier<PathChain> pathChain;


        // PID constants for turret
        private double Kp = 0.027;
        private double turretOutput;
        private boolean close = true;
        private boolean pinned = false;
        private boolean far = false;

        private double t1urret;

        double kp = 0.004;
        double kd = 0.05;


        private double slowModeMultiplier = 2000;
        private double offset = 0;
        private double IMU_reset_var = 0;
        private boolean heading_pid;
        private double heading_correction = 0;

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
        private void setHeading_pid(){

        }
        private void feildCentricDrive(double pinpointDistance) {
            double distanceToTarget = pinpointDistance - IMU_reset_var;
            if (pinpointDistance < 0) {
                distanceToTarget += (2 * (Math.PI));
            }

            double y = Math.cos(distanceToTarget) * (gamepad1.right_stick_y) + (gamepad1.right_stick_x) * (Math.sin(distanceToTarget));
            double x = Math.cos(distanceToTarget) * (gamepad1.right_stick_x) - (gamepad1.right_stick_y) * (Math.sin(distanceToTarget));
            double correction = (gamepad1.left_stick_x)/2;

            ((DcMotorEx) FrontleftdrivemotorBRFL).setVelocity(((y - x) - correction) * slowModeMultiplier);
            ((DcMotorEx) FrontrightdrivemotorBLORF).setVelocity(((y + x) + correction) * slowModeMultiplier);
            ((DcMotorEx) backleftdrivemotorBLORF).setVelocity(((y + x) - correction) * slowModeMultiplier);
            ((DcMotorEx) backrightdrivemotorBRFL).setVelocity(((y - x) + correction) * slowModeMultiplier);


        }
        private void pinpointautoaim() {
            System.out.println(counterturret.getPosition());
            Pose2D pos2D = pinpoint.getPosition();
            double turretX = pos2D.getX(DistanceUnit.MM) + (Math.cos(pos2D.getHeading(AngleUnit.RADIANS)));
            double turretY = pos2D.getY(DistanceUnit.MM) + (Math.sin(pos2D.getHeading(AngleUnit.RADIANS)));
            double distY = (3657.6 - Math.abs(turretY))/25.4;
            double distX = (3657.6 - Math.abs(turretX))/25.4;
            double fieldAngleRad = Math.atan2(distY, distX);
            double fieldAngleDeg = Math.toDegrees(fieldAngleRad);
            double robotHeadingDeg = -pos2D.getHeading(AngleUnit.DEGREES);
            double turretTargetDeg = fieldAngleDeg - robotHeadingDeg;
            double turned = turretTargetDeg;
            counterturret.setPosition(0.467-turned/600);
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
            shooter1Motor.setDirection(DcMotor.Direction.FORWARD);
            shooter1Motor.setVelocityPIDFCoefficients(0.2, 0, 0, 0.5);
            shooter2Motor = hardwareMap.get(DcMotorEx.class, "RightShooter_Motor");
            shooter2Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2Motor.setDirection(DcMotor.Direction.FORWARD);
            shooter2Motor.setVelocityPIDFCoefficients(0.2, 0, 0, 0.5);

            // initializing transfer motors
            transferMotor = hardwareMap.get(DcMotor.class, "Intake_Motor");
            transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            transferMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            // initializing servo
            counterturret = hardwareMap.servo.get("counterturret");

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
        }

        @Override
        public void loop() {
            // PID battery stuff
            double svoltage = getBatteryVoltage();
            double sKF = f / svoltage;


            // turret control
            //autoaim();
            //pinpointautoaim();
            // intake motor control

            transferMotor.setPower(1);

            // transfer motor control

            if (gamepad1.aWasPressed()){
                p += 0.5;
            }
            if (gamepad1.bWasPressed()) {
                p -= 0.5;
            }

            if (gamepad2.xWasPressed()){
                f += 0.5;
            }
            if (gamepad2.yWasPressed()) {
                f -= 0.5;
            }

            shooter1Motor.setVelocityPIDFCoefficients(p, 0, 0, sKF);
            shooter2Motor.setVelocityPIDFCoefficients(p, 0, 0, sKF);
            shooter1Motor.setVelocity(-1500);
            shooter2Motor.setVelocity(1500);
            telemetry.addData("Shooterp: ", p);
            telemetry.addData("Shooterd: ", d);
            telemetry.addData("Shooterf: ", f);
            telemetry.addData("Target Velocity: ", 1500);
            telemetry.addData("Real Velocity1: ", shooter1Motor.getVelocity());
            telemetry.addData("Real Velocity2: ", shooter2Motor.getVelocity());
            //telemetry.addData("servo1Pos", p);
            //telemetry.addData("servo2Pos", s);
        }
        public void configurePinpoint () {
            pinpoint.setOffsets(-160.0, -50.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

            pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.FORWARD);

            pinpoint.resetPosAndIMU();

        }
    }