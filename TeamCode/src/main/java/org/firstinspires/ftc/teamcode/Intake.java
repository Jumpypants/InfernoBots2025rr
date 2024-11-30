package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake {
    public static double WRIST_ROTATE_TIME = 1;

    public static double WRIST_DOWN_POSITION = 1;
    public static double WRIST_MID_POSITION = 0.7;
    public static double WRIST_UP_POSITION = 0.3;

    public static double SLIDE_IN_POSITION = 4;
    public static double TRANSFER_TIME = 1;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double CLAW_OPEN_POSITION = 0.22;
    public static double CLAW_CLOSED_POSITION = 0.0;

    public static double EXTEND_TO_SAMPLE_OFFSET = -2;
    public static double SLIDE_TICKS_PER_INCH = 0.0454545;
    public static double ALLOWED_ERROR = 1;

    public static double KP = 0.028;
    public static double KI = 0.0;
    public static double KD = 0.0;

    private final Motor SLIDE_MOTOR;
    private final Servo WRIST_SERVO;
    private final Servo CLAW_SERVO;

    private double slidePosition = 0;

    public Intake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "intakeSlide");
        SLIDE_MOTOR.resetEncoder();
        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        SLIDE_MOTOR.setInverted(true);

        WRIST_SERVO = hardwareMap.get(Servo.class, "intakeWrist");
        CLAW_SERVO = hardwareMap.get(Servo.class, "intakeClaw");
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


    public void setWrist (double p) {
//        if (p > WRIST_MAX_POSITION) {
//            p = WRIST_MAX_POSITION;
//        } else if (p < WRIST_MIN_POSITION) {
//            p = WRIST_MIN_POSITION;
//        }
        WRIST_SERVO.setPosition(p);
    }

    public double getSlidePosition() {
        slidePosition = (double) (SLIDE_MOTOR.getCurrentPosition()) * SLIDE_TICKS_PER_INCH;
        return slidePosition;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR;
    }

    public void setClaw (double p) {
        CLAW_SERVO.setPosition(p);
    }
}
