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


    public Command localize;

    @Override
    public void initialize() {
        ServoEx = new ServoEx("cr_servo_name");
        imu = new IMUEx("imu", Direction.RIGHT, Direction.UP).zeroed();
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

    public <position> void turret_on(double position) {
        ServoEx.setPosition(position);
    }
    public void turret_off() {
        ServoEx.setPosition(0.5);
    }

    public void calculate_heading(Pose currentpose) {
        double turretX = currentpose.getX() + (Math.cos(currentpose.getHeading()));
        double turretY = currentpose.getY() + (Math.sin(currentpose.getHeading()));
        double distY = (3657.6 - Math.abs(turretY))/25.4;
        double distX = (3657.6 - Math.abs(turretX))/25.4;
        double fieldAngleRad = Math.atan2(distY, distX);
        double robotHeadingRadians = (-currentpose.getHeading());
        double turretTargetDeg = fieldAngleRad - robotHeadingRadians;
        double turnneed = 0.5 - turretTargetDeg/(Math.PI*2);
        ActiveOpMode.telemetry().addData("position", turnneed);
        turret_on(turnneed);

    }


    @Override
    public void periodic() {

    }

}