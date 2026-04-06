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
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;


@Configurable
public class TurretSubsystem implements Subsystem {

    public final TurretSubsystem INSTANCE = new TurretSubsystem();
    public TurretSubsystem() {
    }
    private IMUEx imu;
    GoBildaPinpointDriver pinpoint;

    public int alliance;
    ServoEx ServoEx;



    @Override
    public void initialize() {
        ServoEx = new ServoEx("cr_servo_name");
        imu = new IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed();
    }

    public void turret_on(double position) {
        ServoEx.setPosition(position);
    }
    public void turret_off() {
        ServoEx.setPosition(0.5);
    }

    public void calculate_heading(Pose currentpose) {
        double turretX = 0;
        double turretY = 0;
        double distY = 0;
        double distX = 0;
        double turnneed = 0;
        if (isBlue()) {
            turretX = currentpose.getX() + (Math.cos(currentpose.getHeading()));
            turretY = currentpose.getY() + (Math.sin(currentpose.getHeading()));
            distY = 141 - Math.abs(turretY);
            distX = 141 - Math.abs(turretX);
            double fieldAngleRad = Math.atan2(distY, distX);
            double robotHeadingRadians = (-currentpose.getHeading());
            double turretTargetRad = fieldAngleRad - robotHeadingRadians;
            turnneed = 0.5 - turretTargetRad/(Math.PI*2);
            turret_on(turnneed);
        } else if (isRed()){
            turretX = currentpose.getX() + (Math.cos(currentpose.getHeading()));
            turretY = currentpose.getY() + (Math.sin(currentpose.getHeading()));
            distY = 141 - Math.abs(turretY);
            distX = 141 - Math.abs(turretX);
            double fieldAngleRad = Math.atan2(distY, distX);
            double robotHeadingRadians = (-currentpose.getHeading());
            double turretTargetRad = fieldAngleRad - robotHeadingRadians;
            turnneed = 0.5 +  turretTargetRad/(Math.PI*2);
        }
        //turret_on(turnneed);
        ActiveOpMode.telemetry().addData("turnneed", turnneed);
        ActiveOpMode.telemetry().addData("robotx", turretX);
        ActiveOpMode.telemetry().addData("roboty", turretY);
        ActiveOpMode.telemetry().addData("goalx", distY);
        ActiveOpMode.telemetry().addData("goaly", distX);

    }


    @Override
    public void periodic() {

    }

}