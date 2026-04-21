package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem.shooting;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
public class IntakeTransferSubsystem implements Subsystem {
    public IntakeTransferSubsystem() {

    }
    public static final IntakeTransferSubsystem INSTANCE = new IntakeTransferSubsystem();
    static ServoEx blocker;
    static ServoEx light;

    public static boolean BallInIntake = true;
    public static boolean intake = false;
    public static boolean transfer = false;
    public static boolean close = false;
    public static MotorEx transfer_Motor;
    public static MotorEx intake_Motor;
    public static boolean BallInTransfer = true;
    public static boolean BallInThroat = true;
    static ColorRangeSensor Intake;
    static ColorRangeSensor Transfer;
    public static void UpdateColorSensors() {
        BallInIntake = (Intake.getDistance(DistanceUnit.CM) < 5.7);
        BallInTransfer = (Transfer.getDistance(DistanceUnit.CM) < 6.5);
    }
    public static int NumberOfBallsInBobot() {
        UpdateColorSensors();
        int Balls = 0;
        if (BallInThroat) {
            Balls += 1;
            if (BallInTransfer) {
                Balls += 1;
                if (BallInIntake) {
                    Balls += 1;
                }
            }

        }
        //ActiveOpMode.telemetry().addData("balls brosky", BallInIntake);

        // do not take this out of context
        // mhm lil bro

        return Balls;
    }
    public static void runIntake() {
        intake_Motor.setPower(1);
    }
    public static void runTransfer() {
        transfer_Motor.setPower(-1);
    }
    public static void stopIntake() {
        intake_Motor.setPower(0);
    }
    public static void stopTransfer() {
        transfer_Motor.setPower(0);
    }
    public static void blockerOpen() {
        blocker.setPosition(0.6); // todo: find open position of blocker servo
    }
    public static void blockerClose() {
        blocker.setPosition(0.3); // todo: find closed position of blocker servo
    }
    public static void autonomousIntakeTransferOperation(boolean shooting) {
        if (!shooting) {
            switch (NumberOfBallsInBobot()) {
                case 0:
runIntake();
runTransfer();
                    blockerClose();
                    light.setPosition(0.2);
                    break;
                case 1:
runIntake();
runTransfer();
                    blockerClose();
                    light.setPosition(0.2);
                    break;
                case 2:
                    runIntake();
                    stopTransfer();
                    blockerClose();
                    light.setPosition(0.32);
                    break;
                case 3:
                    stopIntake();
                    stopTransfer();
                    blockerOpen();
                    light.setPosition(0.45);
                    break;
            }
        }
        if (shooting) {
            runIntake();
            runTransfer();
            blockerOpen();

        }
    }

    @Override
    public void initialize() {
        Transfer = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, "rampSensor");
        Intake = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, "intakeSensor");
        transfer_Motor = new MotorEx("Transfer_Motor");
        intake_Motor = new MotorEx("Intake_Motor");

        blocker = new ServoEx("blocker");
        light = new ServoEx("light");
        //light.setPosition(0);

        //blocker.setPosition(0);
    }

    //todo: tune colorsensors to be able to detect balls
    //todo: win worlds

    @Override
    public void periodic() {
        ActiveOpMode.telemetry().addData("intake",intake_Motor.getPower() );
        ActiveOpMode.telemetry().addData("transfer", transfer_Motor.getPower());

        ActiveOpMode.telemetry().addData("intantnat",Intake.getDistance(DistanceUnit.CM) );
        ActiveOpMode.telemetry().addData("transfer", Transfer.getDistance(DistanceUnit.CM));
        UpdateColorSensors();


    }
}