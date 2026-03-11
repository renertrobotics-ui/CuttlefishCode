package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13)
            .forwardZeroPowerAcceleration(-31.298)
            .lateralZeroPowerAcceleration(-71.689)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.04,
                    0,
                    0,
                    0
            ))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
                    0.02,
                    0,
                    0.00025,
                    0.01
            ))
            .headingPIDFCoefficients(new PIDFCoefficients(
                    0.7,
                    0,
                    0,
                    0
            ))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
                    0,
                    0.1,
                    0.0005
            ))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.0047,
                    0,
                    0,
                    0.6,
                    0
            ))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.003,
                    0,
                    0.00025,
                    0.6,
                    0.02
            ))
            .drivePIDFSwitch(20)

            .centripetalScaling(0.0005);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("Front left drive motor BRFL")
            .leftRearMotorName("back left drive motor BLORF")
            .rightFrontMotorName("Front right drive motor BLORF")
            .rightRearMotorName("back right drive motor BRFL")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76)
            .yVelocity(56.46);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2)
            .strafePodX(-2)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    /**
     These are the PathConstraints in order:
     tValueConstraint, velocityConstraint, translationalConstraint, headingConstraint, timeoutConstraint,
     brakingStrength, BEZIER_CURVE_SEARCH_LIMIT, brakingStart
     The BEZIER_CURVE_SEARCH_LIMIT should typically be left at 10 and shouldn't be changed.
     */
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            0.4,
            0.4,
            0.009,
            0,
            1,
            10,
            1
    );
    //Add custom localizers or drivetrains here
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}


