package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDController;

@Config
public class IntakeV0 {
    public static double WRIST_DOWN_POSITION = 0.195;
    public static double WRIST_MID_POSITION = 0.51;
    public static double WRIST_UP_POSITION = 0.81;

    public static double SPIN_IN = -1;
    public static double SPIN_OUT = 1;
    public static double SPIN_STOP = 0;

    public static double SLIDE_IN_POSITION = 0;
    public static double TRANSFER_SPIN_TIME = 0.95;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double INITIAL_EXTENSION_DISTANCE = 8;
    public static double SLIDE_TICKS_PER_INCH = 0.0454545;
    public static double ALLOWED_ERROR = 1;

    public static double KP = 0.028;
    public static double KI = 0.0;
    public static double KD = 0.0;

    private final Motor SLIDE_MOTOR;

    private final Servo WRIST_LEFT_SERVO;
    private final Servo WRIST_RIGHT_SERVO;

    private final CRServo SPIN_LEFT_SERVO;
    private final CRServo SPIN_RIGHT_SERVO;

    private final ColorSensor COLOR_SENSOR;
    private final DistanceSensor DISTANCE_SENSOR;


    private double slidePosition = 0;

    public IntakeV0(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "intakeSlide");
        SLIDE_MOTOR.resetEncoder();
        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        SLIDE_MOTOR.setInverted(true);

        WRIST_LEFT_SERVO = hardwareMap.get(Servo.class, "intakeWristLeft");
        WRIST_RIGHT_SERVO = hardwareMap.get(Servo.class, "intakeWristRight");

        SPIN_LEFT_SERVO = new CRServo(hardwareMap, "intakeSpinLeft");
        SPIN_LEFT_SERVO.setRunMode(CRServo.RunMode.RawPower);

        SPIN_RIGHT_SERVO = new CRServo(hardwareMap, "intakeSpinRight");
        SPIN_RIGHT_SERVO.setRunMode(CRServo.RunMode.RawPower);

        COLOR_SENSOR = hardwareMap.get(ColorSensor.class, "colorSensor");
        DISTANCE_SENSOR = hardwareMap.get(DistanceSensor.class, "colorSensor");
    }

    public boolean stepSlideTo(double position, Telemetry telemetry) {
        return stepSlideTo(position, 1, telemetry);
    }

    public boolean stepSlideTo(double position, double powerCoefficient, Telemetry telemetry) {
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

        SLIDE_MOTOR.set(power * powerCoefficient);

        return false;
    }

    public int getColor(Telemetry telemetry) {
        final double SCALE_FACTOR = 255;

        float hsvValues[] = {0F, 0F, 0F};

        Color.RGBToHSV((int) (COLOR_SENSOR.red() * SCALE_FACTOR),
                (int) (COLOR_SENSOR.green() * SCALE_FACTOR),
                (int) (COLOR_SENSOR.blue() * SCALE_FACTOR),
                hsvValues);

        telemetry.addData("color", Color.HSVToColor(hsvValues));
        return Color.HSVToColor(hsvValues);
    }

    public void setSpin (double p) {
        SPIN_LEFT_SERVO.set(p);
        SPIN_RIGHT_SERVO.set(-p);
    }

    public void setWrist (double p) {
        WRIST_LEFT_SERVO.setPosition(p);
        WRIST_RIGHT_SERVO.setPosition(1 - p);
    }

    public double getSlidePosition() {
        slidePosition = (double) (SLIDE_MOTOR.getCurrentPosition()) * SLIDE_TICKS_PER_INCH;
        return slidePosition;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR;
    }

    public CRServo getSpinServo() {
        return SPIN_LEFT_SERVO;
    }
}
