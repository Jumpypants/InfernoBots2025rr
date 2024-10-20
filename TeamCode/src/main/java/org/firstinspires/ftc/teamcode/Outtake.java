package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Outtake {
    private final Motor SLIDE_MOTOR;
    private final ServoEx ROTATE_SERVO;

    public static double SLIDE_ROTATIONS_PER_INCH = 1.0;
    public static double ALLOWED_ERROR = 0.1;

    public static double OUT_POSITION = 0.0;
    public static double IN_POSITION = Math.PI / 2;

    public static double KP = 0.05;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.0;

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "outtakeSlide");
        ROTATE_SERVO = new SimpleServo(hardwareMap, "outtakeSpin", 0, Math.PI / 2);
    }

    public boolean stepSlideTo(double position) {
        double slidePosition = SLIDE_MOTOR.getCurrentPosition() * SLIDE_ROTATIONS_PER_INCH;

        if (Math.abs(slidePosition - position) < ALLOWED_ERROR) {
            SLIDE_MOTOR.set(0);
            return true;
        }

        PIDFController pidfController = new PIDFController(KP, KI, KD, KF);

        pidfController.setSetpoint(position);

        double power = pidfController.calculate(slidePosition);

        SLIDE_MOTOR.set(power);

        return false;
    }

    public void stepTurnOut() {

        ROTATE_SERVO.setPosition(OUT_POSITION);

    }

    public void stepTurnIn() {

        ROTATE_SERVO.setPosition(IN_POSITION);

    }
}
