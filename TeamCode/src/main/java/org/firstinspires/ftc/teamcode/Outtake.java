package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake {
    private final Motor SLIDE_MOTOR;
    private final Servo SPIN_SERVO;

    public static double SLIDE_INCH_PER_ROTATION = 4.724;
    public static double ALLOWED_ERROR = 1;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    private double slidePosition = 0;

    public static double SPIN_OUT_POSITION = 0.35;
    public static double SPIN_IN_POSITION = 0.81;
    public static double HIGH_BASKET_POSITION = 41;
    public static double LOW_BASKET_POSITION = 24;
    public static double DOWN_POSITION = 0.0;
    public static double TIME_TO_SPIN = 0.9;

    public static double KP = 0.4;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.21;

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "outtakeSlide");
        SLIDE_MOTOR.setInverted(true);
        SPIN_SERVO = hardwareMap.get(Servo.class, "outtakeSpin");
    }

    public boolean stepSlideTo(double position, Telemetry telemetry) {
        slidePosition = getSlidePosition();

        telemetry.addData("Outtake pos", slidePosition);

        if (position > SLIDE_MAX_POSITION) {
            position = SLIDE_MAX_POSITION;
        } else if (position < SLIDE_MIN_POSITION) {
            position = SLIDE_MIN_POSITION;
        }

        if (Math.abs(slidePosition - position) < ALLOWED_ERROR) {
            SLIDE_MOTOR.set(KF);
            return true;
        }

        PIDController pidController = new PIDController(KP, KI, KD);

        pidController.setSetpoint(position);

        double power = pidController.calculate(slidePosition) + KF;

        SLIDE_MOTOR.set(power);

        return false;
    }

    public boolean stepSlideTo(double position, TelemetryPacket telemetry) {
        slidePosition = getSlidePosition();

        telemetry.put("Outtake pos", slidePosition);

        if (position > SLIDE_MAX_POSITION) {
            position = SLIDE_MAX_POSITION;
        } else if (position < SLIDE_MIN_POSITION) {
            position = SLIDE_MIN_POSITION;
        }

        if (Math.abs(slidePosition - position) < ALLOWED_ERROR) {
            SLIDE_MOTOR.set(KF);
            return true;
        }

        PIDController pidController = new PIDController(KP, KI, KD);

        pidController.setSetpoint(position);

        double power = pidController.calculate(slidePosition) + KF;

        SLIDE_MOTOR.set(power);

        return false;
    }

    public void setSpin(double position) {
        SPIN_SERVO.setPosition(position);
    }

    public void setSlidePower (double p) {
        SLIDE_MOTOR.set(p);
    }

    public double getSlidePosition() {
        slidePosition = SLIDE_MOTOR.getCurrentPosition() * SLIDE_INCH_PER_ROTATION / 360;
        return slidePosition;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR;
    }
}
