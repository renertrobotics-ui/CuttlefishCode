package org.firstinspires.ftc.teamcode.subsystems;


import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import dev.nextftc.core.subsystems.Subsystem;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
public class intakeTransferSubsystem implements Subsystem {
    public static MotorEx transfer_Motor = new MotorEx("Transfer_Motor");
    public static MotorEx intake_Motor = new MotorEx("Intake_Motor");
    public static ServoEx blocker = new ServoEx("blocker");
    public intakeTransferSubsystem() {

    }
    public static final intakeTransferSubsystem INSTANCE = new intakeTransferSubsystem();
    public static boolean BallInIntake = true;
    public static boolean BallInTransfer = true;
    public static boolean BallInThroat = true;
    static ColorRangeSensor Intake;
    static ColorRangeSensor Transfer;
    @Override
    public void initialize() {
        Transfer = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, "rampSensor");
        Intake = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class, "intakeSensor");
    }

    //todo: tune colorsensors to be able to detect balls
    //todo: win worlds

    public static void UpdateColorSensors() {
        BallInIntake = (Intake.getDistance(DistanceUnit.CM) < 6.2);

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
        ActiveOpMode.telemetry().addData("balls brosky", BallInIntake);

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
        blocker.setPosition(0.5); // todo: find open position of blocker servo
    }
    public static void blockerClose() {
        blocker.setPosition(0.5); // todo: find closed position of blocker servo
    }
    public static void autonomousIntakeTransferOperation() {
        switch (NumberOfBallsInBobot()) {
            case 0:
                runIntake();
                runTransfer();
                //blockerClose();
                break;
            case 1:
                runIntake();
                runTransfer();
                //blockerClose();
                break;
            case 2:
                runIntake();
                stopTransfer();
                //blockerOpen();
                break;
            case 3:
                stopIntake();
                stopTransfer();
                //blockerOpen();
                break;
        }
    }
    @Override
    public void periodic() {
        UpdateColorSensors();
        autonomousIntakeTransferOperation();
        ActiveOpMode.telemetry().addData("itakecolorsensorreading", (Intake.getDistance(DistanceUnit.CM)));
        ActiveOpMode.telemetry().addData("newcolorsensorreading", (Transfer.getDistance(DistanceUnit.CM)));
        ActiveOpMode.telemetry().update();

    }
}