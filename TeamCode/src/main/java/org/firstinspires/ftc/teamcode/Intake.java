package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {
    public static double WRIST_ROTATE_TIME = 2;
    public static double WRIST_DOWN_POSITION = 0.82;
    public static double WRIST_UP_POSITION = 0.225;
    public static double SLIDE_IN_POSITION = 0;
    public static double TRANSFER_SPIN_TIME = 2;

    public static double EXTEND_TO_SAMPLE_OFFSET = -2;
    public static double SLIDE_ROTATIONS_PER_INCH = 4.4 * 3;
    public static double ALLOWED_ERROR = 0.1;

    public static double KP = 0.07;
    public static double KI = 0.0;
    public static double KD = 0.0;

    private final Motor SLIDE_MOTOR;
    private final SimpleServo WRIST_SERVO;
    private final Motor SPIN_MOTOR;

    private double slidePosition = 0;

    public Intake(HardwareMap hardwareMap) {
        this.SLIDE_MOTOR = new Motor(hardwareMap, "intakeSlide");
        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.WRIST_SERVO = new SimpleServo(hardwareMap, "intakeWrist", 0, Math.PI / 2);
        this.SPIN_MOTOR = new Motor(hardwareMap, "intakeSpin");
        SLIDE_MOTOR.setInverted(true);
    }

    public boolean stepSlideTo(double position) {
        slidePosition = getSlidePosition();

        if (Math.abs(slidePosition - position) < ALLOWED_ERROR) {
            SLIDE_MOTOR.set(0);
            return true;
        }

        PIDController pidController = new PIDController(KP, KI, KD);

        pidController.setSetpoint(position);

        double power = pidController.calculate(slidePosition);

        SLIDE_MOTOR.set(power);

        return false;
    }


    public void setWrist (double p) {
        WRIST_SERVO.setPosition(p);
    }

    public void setSpin (double p) {
        SPIN_MOTOR.set(p);
    }

    public double getSlidePosition() {
        slidePosition = (double) (SLIDE_MOTOR.getCurrentPosition()) / 360 * SLIDE_ROTATIONS_PER_INCH;
        return slidePosition;
    }
}
