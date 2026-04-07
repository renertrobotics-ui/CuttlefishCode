package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.nextftc.core.subsystems.Subsystem;
//todo: add imports for color sensors
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import dev.nextftc.ftc.ActiveOpMode;


import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;
import dev.nextftc.hardware.powerable.SetPower;

public class intakeTransferSubsystem implements Subsystem {
    // Hardware


    public static MotorEx transfer_Motor = new MotorEx("Transfer_Motor");

    public static MotorEx intake_Motor = new MotorEx("Intake_Motor");
public static ServoEx blocker = new ServoEx("blocker");
    public intakeTransferSubsystem() {

    }

    public static final intakeTransferSubsystem INSTANCE = new intakeTransferSubsystem();
    public static boolean BallInIntake = true;
    public static boolean BallInTransfer = true;
    public static boolean BallInThroat = true;

    static ColorSensor Intake;
    static ColorSensor Transfer;
@Override
    public void initialize() {
        Transfer = hardwareMap.get(ColorSensor.class, "rampSensor");
        Intake = hardwareMap.get(ColorSensor.class, "intakeSensor");


    }

    //todo: tune colorsensors to be able to detect balls
    //todo: win worlds

    public static void UpdateColorSensors() {
        BallInIntake = (((DistanceSensor) Intake).getDistance(DistanceUnit.CM) < 12);
        BallInTransfer = (((DistanceSensor) Transfer).getDistance(DistanceUnit.CM) < 12);
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

        return Balls;
    }
    public static void runIntake() {
        intake_Motor.setPower(1);
    }
    public static void runTransfer() {
        transfer_Motor.setPower(1);
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
        ActiveOpMode.telemetry().update();

    }
}