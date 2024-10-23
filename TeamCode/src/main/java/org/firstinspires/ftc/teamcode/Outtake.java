package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Outtake {
    private final Motor SLIDE_MOTOR;
    private final ServoEx ROTATE_SERVO;

    public static double SLIDE_ROTATIONS_PER_INCH = 4.724;
    public static double ALLOWED_ERROR = 0.1;

    private double slidePosition = 0;

    public static double OUT_POSITION = 0.0;
    public static double IN_POSITION = Math.PI / 2;

    public static double KP = 0.07;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.205;

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "outtakeSlide");
        SLIDE_MOTOR.setInverted(true);
        ROTATE_SERVO = new SimpleServo(hardwareMap, "outtakeSpin", 0, Math.PI / 2);
    }

    public boolean stepSlideTo(double position, Telemetry telemetry) {
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

    public void stepTurnOut() {

        ROTATE_SERVO.setPosition(OUT_POSITION);

    }

    public void stepTurnIn() {

        ROTATE_SERVO.setPosition(IN_POSITION);

    }

    public Motor getSLIDE_MOTOR() {
        return SLIDE_MOTOR;
    }

    public double getSlidePosition() {
        slidePosition = SLIDE_MOTOR.getCurrentPosition() * SLIDE_ROTATIONS_PER_INCH / 360;
        return slidePosition;
    }
}
