package org.firstinspires.ftc.teamcode.subsystems;

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

    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

    // Hardware
    public static MotorEx flywheel = new MotorEx("LeftShooter_Motor");
    public static MotorEx flywheel2 = new MotorEx("RightShooter_Motor");
    public static double kp = 0.015;
    public static double ki = 0.005;
    public static double kd = 0;
    public static double kv = 0;

    // Static Controllers (Created once to preserve PID/Integral memory)
    private static final ControlSystem controller1 = ControlSystem.builder()
            .velPid(new PIDCoefficients(0.007, 0, 0))
            .basicFF(new BasicFeedforwardParameters(0.000368, 0, 0.0))
            .build();

    private static final ControlSystem controller2 = ControlSystem.builder()
            .velPid(new PIDCoefficients(0.007, 0, 0))
            .basicFF(new BasicFeedforwardParameters(0.000368, 0, 0.0))
            .build();

    public static double flywheelvelocity;
    public static double flywheelvelocity2;

    // Track the desired velocity across loops
    private static float targetVelocity = 0;

    /**
     * Sets the desired velocity for the flywheels.
     * The periodic loop will use PIDF to maintain this speed.
     */
    public void setVelocity(float tps) {
        targetVelocity = tps;
    }

    // 1. Keep original name: Left side control
    public static void velocityControlWithFeedforward(KineticState currentstate, double configtps) {
        controller1.setGoal(new KineticState(0.0, configtps, 0.0));
        double power = controller1.calculate(currentstate);
        flywheel.setPower(power);
    }

    // 2. Keep original name: Right side control
    public static void velocityControlWithFeedforward2(KineticState currentstate, double configtps) {
        controller2.setGoal(new KineticState(0.0, configtps, 0.0));
        double power = controller2.calculate(currentstate);
        flywheel2.setPower(-power); // Inverted for hardware
    }

    // 3. Keep original name: Main update loop
    public static void shooter(double tps) {
        BindingManager.update();

        flywheelvelocity = flywheel.getVelocity();
        flywheelvelocity2 = flywheel2.getVelocity();
        double Fvalue = tps/3000;
        double Pvalue = ((tps - flywheelvelocity) / 200);
        double power = Fvalue + Pvalue;

        flywheel.setPower(power);
        flywheel2.setPower(-power);
        //ActiveOpMode.telemetry().addData("position left", flywheel.getVelocity());
        //ActiveOpMode.telemetry().addData("position right", flywheel2.getVelocity());


        // Pass current velocities to the controllers
        //velocityControlWithFeedforward(new KineticState(0, flywheelvelocity, 0), tps);
        //velocityControlWithFeedforward2(new KineticState(0, -flywheelvelocity2, 0), tps);
    }

    @Override
    public void initialize() {
        // Ensure motors are in a neutral state on start
        targetVelocity = 0;
    }

    @Override
    public void periodic() {
       // ActiveOpMode.telemetry().addData("position left", flywheel.getVelocity());
        //ActiveOpMode.telemetry().addData("position right", flywheel2.getVelocity());
        //ActiveOpMode.telemetry().update();

        // Continuously run the shooter logic at the current target velocity
    }
}