package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.teleop.blueteleop.isBlue;
import static org.firstinspires.ftc.teamcode.opmodes.teleop.redteleop.isRed;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
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
    public static final MotorEx throughbore = new MotorEx("Transfer_Motor");

    public static double current_servo_position = 0.5;
    public boolean operator_on = true;

    public int alliance;
    static CRServoEx ServoExLeft;
    static CRServoEx ServoExRight;
    public Command localize;

    public static double RawEncoderValue;
    public double PreviousTurretPos;
    public static PIDCoefficients myPidCoeff = new PIDCoefficients(0.00006, 0, 0.0003);
//    public static BasicFeedforwardParameters myFF = new BasicFeedforwardParameters(0.0, 0, 0.0);




    private ControlSystem controller2;


    Pose startingpose = new Pose(72,72, Math.toRadians(90));

    @Override
    public void initialize() {

        ServoExLeft = new CRServoEx("axonLeft");
        ServoExRight = new CRServoEx("axonRight");
        imu = new IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed();
    }

    public static double GetTurretPosInRadians(){
        RawEncoderValue = throughbore.getCurrentPosition();
        ActiveOpMode.telemetry().addData("turretpos", RawEncoderValue);
        ActiveOpMode.telemetry().addData("adjusted", RawEncoderValue * (Math.PI / 12288));

        return RawEncoderValue;
    }

    public static void velocityControlWithFeedforwardExample2(KineticState currentstate, double targetpos) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller2 = ControlSystem.builder()
                .velPid(myPidCoeff) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .build();

        controller2.setGoal(new KineticState(targetpos));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double turretF = 0.1;
        double piwer = Math.abs(controller2.calculate(currentstate))/controller2.calculate(currentstate);
        double power = controller2.calculate(currentstate) + turretF*(Math.signum(targetpos-GetTurretPosInRadians()));
        ServoExRight.setPower(power);
        ServoExLeft.setPower(power);
    }
    public static void turret_on_via_encoder_and_crservos(double target){// will send turret to target position, with target in radian
        BindingManager.update();
        double turretpos = GetTurretPosInRadians();
        double delta = turretpos - INSTANCE.PreviousTurretPos;
        double f;
        KineticState currentstate = new KineticState(turretpos, delta, 0.0);
        ControlSystem controller2 = ControlSystem.builder()
                .posPid(myPidCoeff) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .build();
        controller2.setGoal(new KineticState(target, 0, 0));
        if (Math.abs(target-turretpos) < 500) {
            f = Math.signum(target-turretpos) * Math.abs(target-turretpos) / 500;
        } else {
            f = Math.signum(target-turretpos);
        }
        double turretF = 0.115;
        ActiveOpMode.telemetry().addData("controller2.calculate(currentstate)", controller2.calculate(currentstate));
        double power = controller2.calculate(currentstate) + turretF*f;
        INSTANCE.PreviousTurretPos = turretpos;
        ServoExRight.setPower(-power);
        ServoExLeft.setPower(-power);

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
        //ActiveOpMode.telemetry().addData("position left", ServoExLeft.getPosition());
        //ActiveOpMode.telemetry().addData("position right", ServoExRight.getPosition());
        ActiveOpMode.telemetry().update();


    }

}