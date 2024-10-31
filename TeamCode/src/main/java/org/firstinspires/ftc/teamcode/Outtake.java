package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake {
    public static enum State {
        GoDown,
        OuttakeHigh,
        Stopped
    }

    private final Motor SLIDE_MOTOR;
    private final ServoEx SPIN_SERVO;

    public static double SLIDE_ROTATIONS_PER_INCH = 4.724;
    public static double ALLOWED_ERROR = 0.1;

    private double slidePosition = 0;

    public static double SPIN_OUT_POSITION = 0.0;
    public static double SPIN_IN_POSITION = Math.PI / 2;
    public static double HIGH_OUTTAKE_POSITION = 42;
    public static double DOWN_POSITION = 0.0;
    public static double TIME_TO_SPIN = 2;

    public boolean finishedOuttake = false;

    private State state = State.Stopped;

    private final ElapsedTime timeSpinning = new ElapsedTime();

    public static double KP = 0.07;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.205;

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "outtakeSlide");
        SLIDE_MOTOR.setInverted(true);
        SPIN_SERVO = new SimpleServo(hardwareMap, "outtakeSpin", 0, Math.PI / 2);
    }

    private boolean stepSlideTo(double position, Telemetry telemetry) {
        slidePosition = getSlidePosition();

        if (Math.abs(slidePosition - position) < ALLOWED_ERROR) {
            SLIDE_MOTOR.set(0);
            return true;
        }

        PIDController pidController = new PIDController(KP, KI, KD);

        pidController.setSetpoint(position);

        double power = pidController.calculate(slidePosition) + KF;
        telemetry.addData("Slide Power", power);
        telemetry.addData("KP", KP);

        SLIDE_MOTOR.set(power);

        return false;
    }

    public void step(Telemetry telemetry) {
        switch (state) {
            case GoDown:
                if (stepSlideTo(DOWN_POSITION, telemetry)) {
                    setSpin(SPIN_IN_POSITION);
                    setState(State.Stopped);
                }
                break;
            case OuttakeHigh:
                finishedOuttake = false;
                if (stepSlideTo(HIGH_OUTTAKE_POSITION, telemetry)) {
                    setSpin(SPIN_OUT_POSITION);
                    if (timeSpinning.seconds() >= TIME_TO_SPIN) {
                        setState(State.Stopped);
                        finishedOuttake = true;
                    }
                }
                break;
            case Stopped:
                SLIDE_MOTOR.set(0);
                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    public void setSpin(double position) {
        SPIN_SERVO.setPosition(position);
    }

    public double getSlidePosition() {
        slidePosition = SLIDE_MOTOR.getCurrentPosition() * SLIDE_ROTATIONS_PER_INCH / 360;
        return slidePosition;
    }
}
