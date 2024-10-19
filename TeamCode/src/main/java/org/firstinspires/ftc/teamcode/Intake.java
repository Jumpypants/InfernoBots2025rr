package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {
    public static double ROTATIONS_PER_INCH = 1.0;
    public static double ALLOWED_ERROR = 0.1;

    public static double KP = 0.1;
    public static double KI = 0.01;
    public static double KD = 0.001;

    private final Motor SLIDE_MOTOR;

    private double slidePosition = 0.0;

    public Intake(HardwareMap hardwareMap) {
        this.SLIDE_MOTOR = new Motor(hardwareMap, "intake");
    }

    public boolean stepSlideTo(double position) {
        slidePosition = SLIDE_MOTOR.getCurrentPosition() * ROTATIONS_PER_INCH;

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

    public double getSlidePosition() {
        return slidePosition;
    }
}
