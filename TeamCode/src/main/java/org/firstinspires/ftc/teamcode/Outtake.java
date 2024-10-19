package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Outtake {
    private final Motor SLIDE_MOTOR;
    private final CRServo ROTATE_SERVO;

    public static double SLIDE_ROTATIONS_PER_INCH = 1.0;
    public static double ALLOWED_ERROR = 0.1;

    public static double OUT_POSITION = 0.0;
    public static double IN_POSITION = Math.PI / 2;

    public static double KP = 0.1;
    public static double KI = 0.01;
    public static double KD = 0.001;
    public static double KF = 0.0;

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "outtake");
        ROTATE_SERVO = new CRServo(hardwareMap, "rotate");
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

    public boolean stepTurnOut() {
        if (ROTATE_SERVO.getCurrentPosition() <= OUT_POSITION) {
            return true;
        }

        ROTATE_SERVO.set(-1);

        return false;
    }

    public boolean stepTurnIn() {
        if (ROTATE_SERVO.getCurrentPosition() >= IN_POSITION) {
            return true;
        }

        ROTATE_SERVO.set(1);

        return false;
    }

    public void stopServo() {
        ROTATE_SERVO.set(0);
    }
}
