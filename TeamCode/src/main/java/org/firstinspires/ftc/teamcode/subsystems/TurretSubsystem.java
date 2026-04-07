package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.teleop.blueteleop.isBlue;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.redteleop.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.FieldCentric;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.ServoEx;
import com.qualcomm.robotcore.hardware.Servo;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class TurretSubsystem implements Subsystem {

    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    public TurretSubsystem() {
    }
    private IMUEx imu;
    GoBildaPinpointDriver pinpoint;
    public double current_servo_position = 0.5;
    public boolean operator_on = true;

    public int alliance;
    ServoEx ServoExLeft;
    ServoEx ServoExRight;
    public Command localize;



    Pose startingpose = new Pose(72,72, Math.toRadians(90));

    @Override
    public void initialize() {
        if(isBlue()!=true && isRed()!=true) {
            ActiveOpMode.telemetry().addLine("No direction set");
        }
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
        ServoExLeft = new ServoEx("axonLeft");
        ServoExRight = new ServoEx("axonRight");
        imu = new IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed();
    }

    public void turret_on(double position) {
        ActiveOpMode.telemetry().addData("left position automatic", 0.5 + position);
        ServoExLeft.setPosition(0.5 + position); // 0.5 is center position
        ServoExRight.setPosition(0.5 - position); // invert to make sure both end up in the same spot
        current_servo_position = ServoExLeft.getPosition() - 0.5;
    }
    public void turret_off() {
        ActiveOpMode.telemetry().addData("left position heading", 0.5);
        //ServoExLeft.setPosition(0.5);
        //ServoExRight.setPosition(0.5);
        current_servo_position = ServoExLeft.getPosition() - 0.5;
    }

    public void operator_control(double positionChange) {
        current_servo_position += positionChange/50;
        ActiveOpMode.telemetry().addData("left position operator", 0.5 + current_servo_position);
        //ServoExLeft.setPosition(0.5 + current_servo_position);
        //ServoExRight.setPosition(0.5 - current_servo_position);
    }
    public Command Localize(){
        return localize;
    }

    public void calculate_heading(Pose currentpose) {
        double turretX = 0;
        double turretY = 0;
        double distY = 0;
        double distX = 0;
        double turnneed = 0;
        if (isBlue()) {
            distY = 141 - currentpose.getY();
            distX = 141 - currentpose.getX();
            double fieldAngleRad = Math.atan2(distY, distX);
            double robotHeadingRadians = (-currentpose.getHeading());
            double turretTargetRad = fieldAngleRad - robotHeadingRadians;
            turnneed = turretTargetRad/(Math.PI*2);
        }
        if (isRed()){
            distY = 141 - currentpose.getY();
            distX = 141 - currentpose.getX();
            double fieldAngleRad = Math.atan2(distY, distX);
            double robotHeadingRadians = (-currentpose.getHeading());
            double turretTargetRad = fieldAngleRad - robotHeadingRadians;
            turnneed = turretTargetRad/(Math.PI*2);
        }
        turret_on(turnneed);
        ActiveOpMode.telemetry().addData("turnneed", turnneed);
        ActiveOpMode.telemetry().addData("robotx", currentpose.getX());
        ActiveOpMode.telemetry().addData("roboty", currentpose.getY());
        ActiveOpMode.telemetry().addData("goalx", distY);
        ActiveOpMode.telemetry().addData("goaly", distX);

    }


    @Override
    public void periodic() {

        follower.update();
        Pose currPose = follower.getPose();
        double robotHeading = follower.getPose().getHeading();
/*
        if (Gamepads.gamepad1().b().toggleOnBecomesTrue().get()) {
            operator_on = !operator_on;
        }

        if (operator_on){
            if (Gamepads.gamepad1().a().get()) {
                turret_off();
            } else {
                operator_control(Gamepads.gamepad1().leftStickX().get());
            }
        } else {
            calculate_heading(currPose);
        }
*/
        calculate_heading(currPose);
        ActiveOpMode.telemetry().addData("position left", ServoExLeft.getPosition());
        ActiveOpMode.telemetry().addData("position right", ServoExRight.getPosition());
        ActiveOpMode.telemetry().update();


    }

}