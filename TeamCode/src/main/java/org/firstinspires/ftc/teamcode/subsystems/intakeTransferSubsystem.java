package org.firstinspires.ftc.teamcode.subsystems;
import dev.nextftc.core.subsystems.Subsystem;
//todo: add imports for color sensors
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;


public class intakeTransferSubsystem implements Subsystem {
    // Hardware
    NormalizedColorSensor Intake;
    ColorSensor Transfer;


    DcMotorEx  transfer_Motor, intake_Motor ;
    Servo blocker;
    public intakeTransferSubsystem() {

    }

    public static final intakeTransferSubsystem INSTANCE = new intakeTransferSubsystem();
    public static boolean BallInIntake = true;
    public static boolean BallInTransfer = true;
    public static boolean BallInThroat = true;


    Transfer = hardwareMap.get(ColorSensor.class, "c1");
    Intake = hardwareMap.get(NormalizedColorSensor.class, "c2");

    transfer_Motor = hardwareMap.get(DcMotorEx.class, "Transfer_Motor");
    intake_Motor = hardwareMap.get(DcMotorEx.class, "Intake_Motor");

    blocker = hardwareMap.get(Servo.class, "blocker");

    //todo: tune colorsensors to be able to detect balls
    //todo: win worlds

    public void UpdateColorSensors() {
        BallInIntake = (((DistanceSensor) Inkate).getDistance(DistanceUnit.CM) < 12);
        BallInTransfer = (((DistanceSensor) Transfer).getDistance(DistanceUnit.CM) < 12);
    }
    public int NumberOfBallsInBobot() {
        UpdateColorSensors();
        private int Balls = 0;
        if BallInThroat {
            Balls += 1;
            if BallInTransfer {
                Balls += 1;
                if BallInIntake {
                    Balls += 1;
                }
            }

        }
        // do not take this out of context

        return Balls;
    }
    public void runIntake() {
        intake_Motor.setPower(1);
    }
    public void runTransfer() {
        Transfer_Motor.setPower(1);
    }
    public void stopIntake() {
        intake_Motor.setPower(0);
    }
    public void stopTransfer() {
        Transfer_Motor.setPower(0);
    }
    public void blockerOpen() {
        blocker.setPosition(67); // todo: find open position of blocker servo
    }
    public void blockerClose() {
        blocker.setPosition(67); // todo: find closed position of blocker servo
    }
    public void autonomousIntakeTransferOperation{
        switch (NumberOfBallsInBobot()) {
            case 0:
                runIntake();
                runTransfer();
                blockerClose();
                break;
            case 1:
                runIntake();
                runTransfer();
                blockerClose();
                break;
            case 2;
                runIntake();
                stopTransfer();
                blockerOpen();
                break;
            case 3;
                stopIntake();
                stopTransfer();
                blockerOpen();
                break;
        }
    }
    static Command intakeAutonomously = new LambdaCommand()
            .setStart(()-> autonomousIntakeTransferOperation())
            .setIsDone(() -> true);
}