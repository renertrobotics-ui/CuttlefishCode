package org.firstinspires.ftc.teamcode.opmodes.auto;

//import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;
//import static org.firstinspires.ftc.teamcode.subsystems.ShooterCalc.calculateShotVectorandUpdateHeading;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.UpdateColorSensors;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.autonomousIntakeTransferOperation;
import static org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem.autonomousIntakeTransferOperationforAutonomous;
import static org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem.shooter;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.calculate_heading;
import static org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem.turret_on_via_encoder_and_crservos;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.subsystems.DistanceRed;
//import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;


@Autonomous
@Configurable
public class red18ball extends NextFTCOpMode {
    public red18ball(){
        addComponents(
                new SubsystemComponent(ShooterSubsystem.INSTANCE, IntakeTransferSubsystem.INSTANCE, TurretSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(hwMap -> Constants.createFollower(hwMap))


        );
    }
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;


    private Paths paths;
    private boolean shoot = false;
    public MotorEx intakeMotor;

    int tagId = 0;

    public Pose start = new Pose(109, 127, Math.toRadians(90));


    private MotorEx transfer1;


    public static MotorEx flywheel2 = new MotorEx("launchingmotor2");

   /* private CRServo hoodServo1n;
    private CRServo hoodServo2n;

    private CRServoEx hoodServo1 = new CRServoEx(() -> hoodServo1n);
    private CRServoEx hoodServo2 = new CRServoEx(() -> hoodServo2n);






    ParallelGroup HoodRunUp=new ParallelGroup(
            new SetPower(hoodServo1,-1),
            new SetPower(hoodServo2,1)
    );

    public ParallelGroup HoodPowerZero=new ParallelGroup(
            new SetPower(hoodServo1,0),
            new SetPower(hoodServo2,0)
    );

    public SequentialGroup HoodUp=new SequentialGroup(
            HoodRunUp,
            new Delay(0.18),
            HoodPowerZero
    );

    ParallelGroup HoodRunDown=new ParallelGroup(
            new SetPower(hoodServo1,1),
            new SetPower(hoodServo2,-1)
    );

    public SequentialGroup HoodDown=new SequentialGroup(
            HoodRunDown,
            new Delay(0.17),
            HoodPowerZero
    );

    public SequentialGroup HoodUpAuto=new SequentialGroup(
            HoodRunUp,
            new Delay(0.12),
            HoodPowerZero
    );*/

    private Boolean preloadspin;

    private double preloadtps;

    private double shoottps;
    //Command findTPSShoot = new LambdaCommand()
    //.setStart(()->shoottps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
    //Command findTPSPreload = new LambdaCommand()
    // .setStart(()->preloadtps = findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));


    public void onInit() {
        telemetry.addLine("Initializing Follower...");

        telemetry.update();
        follower = PedroComponent.follower();


        IMUEx imu = new IMUEx("imu", Direction.LEFT, Direction.BACKWARD).zeroed();

        paths = new Paths(follower);
        intakeMotor = new MotorEx("Intake_Motor");
        transfer1 = new MotorEx("Transfer_Motor");
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        follower.setStartingPose(start);


        pathState = 0;
        telemetry.addLine("Follower + IMU + Odo Pods initialized successfully!");
        telemetry.addLine("Initialization complete!");
        telemetry.update();

        follower.update();


    }



    Command onshoot = new LambdaCommand()
            .setStart(()-> shoot = true);
    Command offshoot = new LambdaCommand()
            .setStart(()-> shoot = false);


    public SequentialGroup shooter = new SequentialGroup(new Delay(0.05), onshoot, new Delay(0.6), offshoot);


    public Command Auto(){
        return new SequentialGroup(

                /*spinupPLEASEEIsagiINEEDTHIS,
                preloadSpun*/
                new FollowPath(paths.preloadLaunch,true,1.0),
                new Delay(0.5),
                shooter,



                new FollowPath(paths.intakeSet2,true,1.0),



                new FollowPath(paths.launchSet2,true,1.0),
                shooter,

                new FollowPath(paths.resetAndIntake1,true,0.95),
                new Delay(1.5),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam1, true, 1.0),
                shooter,


                new FollowPath(paths.resetAndIntake2,true,0.95),
                new Delay(1.5),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam2, true, 1.0),
                shooter,


                new FollowPath(paths.intakeSet1,true,1.0),
                new FollowPath(paths.launchSet1,true,1.0),
                shooter,
                new FollowPath(paths.resetAndIntake2,true,0.95),
                new Delay(1.5),
                //reverseIntakeForMe,
                new FollowPath(paths.launchSpam2, true, 1.0),
                shooter,



                new FollowPath(paths.teleOpPark,true,1.0)
        );
    }

    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        pathTimer.resetTimer();
        /*flywheel.setPower(1);
        flywheel2.setPower(-1);*/

        //shooter(1085);

        //int tag=MotifScanning.INSTANCE.findMotif();
        Auto().schedule();



    }

    @Override
    public void onUpdate(){
        follower.update();
        UpdateColorSensors();
        autonomousIntakeTransferOperationforAutonomous(shoot);
        shooter(1670);
        Pose currPose = follower.getPose();

        turret_on_via_encoder_and_crservos(3100);

        /*if(preloadspinreal) {
            shooter(1080);
        }
        else{
            if (DistanceRed.INSTANCE.getDistanceFromTag() != 0) {
                shooter(findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
                ActiveOpMode.telemetry().addData("Limelight!", findTPS(DistanceRed.INSTANCE.getDistanceFromTag()));
            } else if (DistanceRed.INSTANCE.getDistanceFromTag() == 0 && !intake3&&!intakeSpam) {
                shooter(1065);
            }
            else if(intake3){
                shooter(1070);
            }
            else if(intakeSpam){
                shooter(1065);
            }
        }*/

        double robotHeading = follower.getPose().getHeading();
        //Vector v = new Vector(new Pose(138, 138));
        //Double[] results = calculateShotVectorandUpdateHeading(robotHeading, robotToGoalVector, follower.getVelocity());
        //double flywheelSpeed = results[0];
        //shooter((float) flywheelSpeed+10);
        //double hoodAngle = results[1];
        //hoodToPos(hoodAngle);

    }




    @Override
    public void onStop() {
        follower.breakFollowing();
        telemetry.addLine("Autonomous Stopped.");
        telemetry.update();
    }


    public class Paths {
        public PathChain preloadLaunch;
        public PathChain intakeSet2;
        public PathChain launchSet2;
        public PathChain resetAndIntake1;
        public PathChain moverBacker;
        public PathChain launchSpam1;
        public PathChain resetAndIntake2;
        public PathChain launchSpam2;
        public PathChain intakeSet1;
        public PathChain launchSet1;
        public PathChain intakeSet3;
        public PathChain launchSet3;
        public PathChain teleOpPark;

        public Paths(Follower follower) {
            preloadLaunch = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(109.000, 127.000),
                                    new Pose(95.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            intakeSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.000, 95.000),
                                    new Pose(80.000, 45.000),
                                    new Pose(125.000, 60.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(20))
                    .build();

            launchSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 60.000),
                                    new Pose(95.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(20), Math.toRadians(0))
                    .build();

            resetAndIntake1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.000, 95.000),
                                    new Pose(114.600, 55.000),
                                    new Pose(128.000, 62)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            launchSpam1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.000, 62),
                                    new Pose(100, 67),
                                    new Pose(95.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();

            resetAndIntake2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.000, 95.000),
                                    new Pose(114.600, 55.000),
                                    new Pose(128.000, 62)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(30))
                    .build();

            launchSpam2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(128.000, 62),
                                    new Pose(100, 67),
                                    new Pose(95.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(30), Math.toRadians(0))
                    .build();
            intakeSet1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(95.000, 95.000),
                                    new Pose(90.000, 80.000),
                                    new Pose(125.000, 82.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setVelocityConstraint(1.0)
                    .setTValueConstraint(0.8).build();

            launchSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(125.000, 82.000),
                                    new Pose(95.000, 95.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            intakeSet3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(50.000, 94.000),
                                    new Pose(54.000, 89.500),   // 142 - 88
                                    new Pose(67.000, 28.615),   // 142 - 75
                                    new Pose(12, 32.000)    // 142 - 129.151
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            launchSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12, 32),
                                    new Pose(50.000, 94.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            teleOpPark = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(95.000, 95.000),
                                    new Pose(118.000, 72.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }
}