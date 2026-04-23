package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import static org.firstinspires.ftc.teamcode.subsystems.Calculations.findTPS;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.bindings.*;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.ControlSystem;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class ShooterSubsystem implements Subsystem {

    public ShooterSubsystem() {
    }

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
    public static double flywheelvelocity;
    public static double flywheelvelocity2;
    private ControlSystem controller1;
    private ControlSystem controller2;

    public static MotorEx flywheel = new MotorEx("LeftShooter_Motor");
    public static MotorEx flywheel2 = new MotorEx("RightShooter_Motor");

    public static PIDCoefficients myPidCoeff = new PIDCoefficients(0.015, 0.005, 0.00);
    public static BasicFeedforwardParameters myFF = new BasicFeedforwardParameters(0.00045, 0, 0.0);

    public static double configvelocity = 1400; // far zone - ~1500. near zone - ~1200-1300

    // --- Quartic Regression Coefficients ---
    private static final double a = -3.25025e-7;
    private static final double b = 0.000367273;
    private static final double c = -0.145764;
    private static final double d = 25.48778;
    private static final double e = 3.291;

    public static void velocityControlWithFeedforwardExample(KineticState currentstate, float configtps) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller1 = ControlSystem.builder()
                .velPid(myPidCoeff) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(myFF) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller1.setGoal(new KineticState(0.0, configtps, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller1.calculate(currentstate);
        flywheel.setPower(power);
    }

    public static void velocityControlWithFeedforwardExample2(KineticState currentstate, float configtps) {
        // Create a velocity controller with PID and feedforward
        ControlSystem controller2 = ControlSystem.builder()
                .velPid(myPidCoeff) // Velocity PID with kP=0.1, kI=0.01, kD=0.05
                .basicFF(myFF) // Basic feedforward with kV=0.02, kA=0.0, kS=0.01 //pid tuning
                .build();

        controller2.setGoal(new KineticState(0.0, configtps, 0.0));

        // In a loop (simulated here), you would:
        // Create a KineticState with current position and velocity

        double power = controller2.calculate(currentstate);
        flywheel2.setPower(-1 * power);
    }

    public static void shooter(float tps) {
        BindingManager.update();
        flywheelvelocity = flywheel.getVelocity();
        flywheelvelocity2 = flywheel2.getVelocity();
        KineticState currentState = new KineticState(0, flywheelvelocity, 0.0);
        KineticState currentState2 = new KineticState(0, -1 * flywheelvelocity2, 0.0);
        velocityControlWithFeedforwardExample(currentState, tps);
        velocityControlWithFeedforwardExample2(currentState2, tps);
        double rpm = (flywheelvelocity / 28) * 60.0;
    }

    /**
     * Calculates the required TPS based on the distance using a quartic regression
     * model, and passes it to the shooter.
     * * @param distance The distance to the target.
     */
    public static void shoot(double distance) {
        // y = ax^4 + bx^3 + cx^2 + dx + e
        double targetTps = (a * Math.pow(distance, 4))
                + (b * Math.pow(distance, 3))
                + (c * Math.pow(distance, 2))
                + (d * distance)
                + e;

        // Cast down to float because the shooter() method expects a float parameter
        shooter((float) targetTps);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void periodic() {

    }
}