package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    public static double WRIST_DOWN_POSITION = 0.05;
    public static double WRIST_MID_POSITION = 0.45;
    public static double WRIST_UP_POSITION = 0.75;

    public static double SPIN_IN = 1;
    public static double SPIN_OUT = -1;
    public static double SPIN_STOP = 0;

    public static double SLIDE_IN_POSITION = 2;
    public static double TRANSFER_SPIN_TIME = 0.6;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double INITIAL_EXTENSION_DISTANCE = 8;
    public static double SLIDE_TICKS_PER_INCH = 0.0454545;
    public static double ALLOWED_ERROR = 1;

    public static double KP = 0.028;
    public static double KI = 0.0;
    public static double KD = 0.0;

    private final Motor SLIDE_MOTOR;
    private final Servo WRIST_SERVO;
    private final CRServo SPIN_SERVO;

    private double slidePosition = 0;

    public Intake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "intakeSlide");
        SLIDE_MOTOR.resetEncoder();
        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        SLIDE_MOTOR.setInverted(true);

        WRIST_SERVO = hardwareMap.get(Servo.class, "intakeWrist");
        SPIN_SERVO = new CRServo(hardwareMap, "intakeSpin");
        SPIN_SERVO.setRunMode(CRServo.RunMode.RawPower);
    }

    public boolean stepSlideTo(double position, Telemetry telemetry) {
        if (position > SLIDE_MAX_POSITION) {
            position = SLIDE_MAX_POSITION;
        } else if (position < SLIDE_MIN_POSITION) {
            position = SLIDE_MIN_POSITION;
        }

        slidePosition = getSlidePosition();

        if (Math.abs(slidePosition - position) < ALLOWED_ERROR) {
            SLIDE_MOTOR.set(0);
            return true;
        }

        PIDController pidController = new PIDController(KP / SLIDE_TICKS_PER_INCH, KI, KD);

        pidController.setSetpoint(position);

        double power = pidController.calculate(slidePosition);

        SLIDE_MOTOR.set(power);

        return false;
    }

    public void setSpin (double p) {
        SPIN_SERVO.set(p);
    }

    public void setWrist (double p) {
        WRIST_SERVO.setPosition(p);
    }

    public double getSlidePosition() {
        slidePosition = (double) (SLIDE_MOTOR.getCurrentPosition()) * SLIDE_TICKS_PER_INCH;
        return slidePosition;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR;
    }

    public CRServo getSpinServo() {
        return SPIN_SERVO;
    }
}
