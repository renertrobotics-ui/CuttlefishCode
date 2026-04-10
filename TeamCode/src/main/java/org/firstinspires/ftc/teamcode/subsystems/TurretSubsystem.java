package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.teleop.blueteleop.isBlue;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.redteleop.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;


@Configurable
public class TurretSubsystem implements Subsystem {

    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
    public TurretSubsystem() {
    }
    private IMUEx imu;
    GoBildaPinpointDriver pinpoint;
    public static double current_servo_position = 0.5;
    public boolean operator_on = true;

    public int alliance;
    static ServoEx ServoExLeft;
    static ServoEx ServoExRight;
    public Command localize;

    public double PreviousturretPos;

    public double GetTurretPosInRadians(){
        private double RawEncoderValue;
        //todo: set rawencodervalue to the throughbore encoder value
        return RawEncoderValue * (Math.pi / 12288);
    }



    Pose startingpose = new Pose(72,72, Math.toRadians(90));

    @Override
    public void initialize() {






        ServoExLeft = new ServoEx("axonLeft");
        ServoExRight = new ServoEx("axonRight");
        imu = new IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed();
    }

    public static void turret_on(double position) {
        ActiveOpMode.telemetry().addData("left position automatic", 0.5 + position);
        ServoExLeft.setPosition(0.5 + position); // 0.5 is center position
        ServoExRight.setPosition(0.5 - position); // invert to make sure both end up in the same spot
        current_servo_position = ServoExLeft.getPosition() - 0.5;
    }
    public static void turret_off() {
        ActiveOpMode.telemetry().addData("left position heading", 0.5);
        //ServoExLeft.setPosition(0.5);
        //ServoExRight.setPosition(0.5);
        current_servo_position = ServoExLeft.getPosition() - 0.5;
    }
    public static void turret_on_via_encoder_and_crservos(double target){// will send turret to target position, with target in radians
        private double Error = target - GetTurretPosInRadians();
        private double P_value = Error * 1;// todo: tune this
        private double D_value = (PreviousturretPos - GetTurretPosInRadians()) * 0;//todo: tune this
        private double Output = P_value + D_value;
        //todo: set axon power accordingly and be careful of negatives
        PreviousturretPos = GetTurretPosInRadians();
    }

    public static void operator_control(double positionChange) {
        current_servo_position += positionChange/50;
        ActiveOpMode.telemetry().addData("left position operator", 0.5 + current_servo_position);
        //ServoExLeft.setPosition(0.5 + current_servo_position);
        //ServoExRight.setPosition(0.5 - current_servo_position);
    }
    public Command Localize(){
        return localize;
    }

    public static double calculate_heading(Pose currentpose) {
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
            ActiveOpMode.telemetry().addData("headingangle", (fieldAngleRad) * 180 / Math.PI );
            turnneed = turretTargetRad/(Math.PI*2);
        }
        if (isRed()){
            distY = 141 - currentpose.getY();
            distX = 141 - currentpose.getX();
            double fieldAngleRad = Math.atan2(distY, distX);
            double robotHeadingRadians = (currentpose.getHeading());
            double turretTargetRad = fieldAngleRad - robotHeadingRadians;
            ActiveOpMode.telemetry().addData("headingangle", (fieldAngleRad) * 180 / Math.PI );

            turnneed = turretTargetRad/(Math.PI*2);
        }


        ActiveOpMode.telemetry().addData("turnneed", turnneed);

        ActiveOpMode.telemetry().addData("heading", (currentpose.getHeading() * 180 / Math.PI ));

        ActiveOpMode.telemetry().addData("roboty", currentpose.getY());
        ActiveOpMode.telemetry().addData("goalx", distY);
        ActiveOpMode.telemetry().addData("goaly", distX);
        return turnneed;

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
        ActiveOpMode.telemetry().addData("position left", ServoExLeft.getPosition());
        ActiveOpMode.telemetry().addData("position right", ServoExRight.getPosition());
        ActiveOpMode.telemetry().update();


    }

}