package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.murphy.MurphyAction;

@Config
public class Outtake {
    private final Motor SLIDE_MOTOR;
    private final Servo SPIN_SERVO;

    public static double SLIDE_INCH_PER_ROTATION = 4.724;
    public static double ALLOWED_ERROR = 2.5;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double SPIN_OUT_POSITION = 0.35;
    public static double SPIN_MID_POSITION = 0.5;
    public static double SPIN_IN_POSITION = 0.81;

    public static double HIGH_BASKET_POSITION = 41;
    public static double LOW_BASKET_POSITION = 24;
    public static double DOWN_POSITION = 0.0;

    public static double KP = 0.35;
    public static double KI = 0.0;
    public static double KD = 0.05;
    public static double KF = 0.21;

    private final PIDFController pidfController = new PIDFController(KP / SLIDE_INCH_PER_ROTATION, KI, KD, KF);

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "outtakeSlide");
        SLIDE_MOTOR.setInverted(true);
        SPIN_SERVO = hardwareMap.get(Servo.class, "outtakeSpin");
    }

    public void setSlideSetPoint(double setPoint) {
        if (setPoint > SLIDE_MAX_POSITION) {
            setPoint = SLIDE_MAX_POSITION;
        } else if (setPoint < SLIDE_MIN_POSITION) {
            setPoint = SLIDE_MIN_POSITION;
        }
        pidfController.setSetpoint(setPoint);
    }

    public void stepSlide() {
        double slidePosition = getSlidePosition();
        double power = pidfController.calculate(slidePosition);
        SLIDE_MOTOR.set(power);
    }

    public boolean isSlideAtSetPoint() {
        double slidePosition = getSlidePosition();
        return Math.abs(slidePosition - pidfController.getSetpoint()) < ALLOWED_ERROR;
    }

    public void setSpin(double position) {
        SPIN_SERVO.setPosition(position);
    }

    public void setSlidePower (double p) {
        SLIDE_MOTOR.set(p);
    }

    public double getSlidePosition() {
        return SLIDE_MOTOR.getCurrentPosition() * SLIDE_INCH_PER_ROTATION / 360;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR;
    }

    public static class MoveSlideAction extends MurphyAction {
        private final Outtake outtake;

        public MoveSlideAction(Outtake outtake, double setPoint) {
            super();
            this.outtake = outtake;
            outtake.setSlideSetPoint(setPoint);
        }

        @Override
        public boolean step() {
            return !outtake.isSlideAtSetPoint();
        }
    }


    public static class SpinToMidAction extends MurphyAction {
        public SpinToMidAction(Outtake outtake) {
            super();
            outtake.setSpin(SPIN_MID_POSITION);
        }

        @Override
        public boolean step() {
            return ELAPSED_TIME.seconds() < 0.5;
        }
    }

    public static class DumpAction extends MurphyAction {
        private final Outtake outtake;
        private final Gamepad gamepad2;

        private boolean buttonPressed = false;

        public DumpAction(Outtake outtake, Gamepad gamepad2) {
            super();
            this.outtake = outtake;
            this.gamepad2 = gamepad2;
        }

        @Override
        public boolean step() {
            if (gamepad2.right_trigger > 0.5) {
                buttonPressed = true;
            }
            if (buttonPressed && gamepad2.right_trigger < 0.1) {
                outtake.setSpin(SPIN_OUT_POSITION);
                return ELAPSED_TIME.seconds() < 0.5;
            }
            return true;
        }
    }
}
