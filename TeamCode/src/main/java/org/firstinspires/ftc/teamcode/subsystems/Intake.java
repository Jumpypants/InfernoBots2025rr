package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.murphy.MurphyAction;

@Config
public class Intake {
    public static double WRIST_DOWN_POSITION = 0.195;
    public static double WRIST_MID_POSITION = 0.51;
    public static double WRIST_UP_POSITION = 0.81;

    public static double SPIN_IN = -1;
    public static double SPIN_OUT = 1;
    public static double SPIN_STOP = 0;

    public static double SLIDE_IN_POSITION = 0;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double INITIAL_EXTENSION_DISTANCE = 8;
    public static double SLIDE_TICKS_PER_INCH = 0.0454545;
    public static double ALLOWED_ERROR = 1;

    public static double KP = 0.028;
    public static double KI = 0.0;
    public static double KD = 0.0;

    private final PIDController pidController = new PIDController(KP / SLIDE_TICKS_PER_INCH, KI, KD);

    private final Motor SLIDE_MOTOR;

    private final Servo WRIST_LEFT_SERVO;
    private final Servo WRIST_RIGHT_SERVO;

    private final CRServo SPIN_LEFT_SERVO;
    private final CRServo SPIN_RIGHT_SERVO;

    private final ColorSensor COLOR_SENSOR;
    private final DistanceSensor DISTANCE_SENSOR;

    public Intake(HardwareMap hardwareMap) {
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

    public void setSlideSetPoint(double setPoint) {
        if (setPoint > SLIDE_MAX_POSITION) {
            setPoint = SLIDE_MAX_POSITION;
        } else if (setPoint < SLIDE_MIN_POSITION) {
            setPoint = SLIDE_MIN_POSITION;
        }
        pidController.setSetpoint(setPoint);
    }

    public void stepSlide(double powerCoefficient) {
        double slidePosition = getSlidePosition();
        double power = pidController.calculate(slidePosition);
        SLIDE_MOTOR.set(power * powerCoefficient);
    }

    public void stepSlide() {
        stepSlide(1);
    }

    public boolean isSlideAtSetPoint() {
        double slidePosition = getSlidePosition();
        return Math.abs(pidController.getSetpoint() - slidePosition) < ALLOWED_ERROR;
    }

    public void setSpin (double p) {
        SPIN_LEFT_SERVO.set(p);
        SPIN_RIGHT_SERVO.set(-p);
    }

    public double getSpin() {
        return SPIN_LEFT_SERVO.get();
    }

    public void setWrist (double p) {
        WRIST_LEFT_SERVO.setPosition(p);
        WRIST_RIGHT_SERVO.setPosition(1 - p);
    }

    public double getWrist() {
        return WRIST_LEFT_SERVO.getPosition();
    }

    public double getSlidePosition() {
        return (double) (SLIDE_MOTOR.getCurrentPosition()) * SLIDE_TICKS_PER_INCH;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR;
    }

    public CRServo getSpinServo() {
        return SPIN_LEFT_SERVO;
    }

    public void driveFieldCentric(double x, double y, double heading) {
        double cosHeading = Math.cos(Math.toRadians(heading));
        double sinHeading = Math.sin(Math.toRadians(heading));

        double power = x * sinHeading + y * cosHeading;

        setSlideSetPoint(getSlidePosition() - power * 0.75);
    }


    public static class WristUpAction extends MurphyAction {
        private final double requiredTime;

        public WristUpAction(Intake intake) {
            super();
            if (intake.getWrist() == WRIST_DOWN_POSITION) {
                requiredTime = 0.5;
            } else if (intake.getWrist() == WRIST_MID_POSITION) {
                requiredTime = 0.25;
            } else {
                requiredTime = 0;
            }
            intake.setWrist(WRIST_UP_POSITION);
        }

        @Override
        public boolean step() {
            return ELAPSED_TIME.seconds() < requiredTime;
        }
    }

    public static class WristDownAction extends MurphyAction {
        private final double requiredTime;

        public WristDownAction(Intake intake) {
            super();
            if (intake.getWrist() == WRIST_UP_POSITION) {
                requiredTime = 0.5;
            } else if (intake.getWrist() == WRIST_MID_POSITION) {
                requiredTime = 0.25;
            } else {
                requiredTime = 0;
            }
            intake.setWrist(WRIST_DOWN_POSITION);
        }

        @Override
        public boolean step() {
            return ELAPSED_TIME.seconds() < requiredTime;
        }
    }

    public static class WristMidAction extends MurphyAction {
        private final double requiredTime;

        public WristMidAction(Intake intake) {
            super();
            if (intake.getWrist() == WRIST_UP_POSITION || intake.getWrist() == WRIST_DOWN_POSITION) {
                requiredTime = 0.25;
            } else {
                requiredTime = 0;
            }
        }

        @Override
        public boolean step() {
            return ELAPSED_TIME.seconds() < requiredTime;
        }
    }

    public static class MoveSlideAction extends MurphyAction {
        private final Intake intake;

        public MoveSlideAction(Intake intake, double setPoint) {
            super();
            this.intake = intake;
            intake.setSlideSetPoint(setPoint);
        }

        @Override
        public boolean step() {
            return !intake.isSlideAtSetPoint();
        }
    }

    public static class TransferSpinAction extends MurphyAction {
        public TransferSpinAction(Intake intake) {
            super();
            intake.setSpin(SPIN_IN);
        }

        @Override
        public boolean step() {
            return ELAPSED_TIME.seconds() < 1;
        }
    }

    public static class ClearWristAction extends MurphyAction {
        public ClearWristAction(Intake intake) {
            super();
            intake.setWrist(WRIST_MID_POSITION);
        }

        @Override
        public boolean step() {
            return ELAPSED_TIME.seconds() < 0.15;
        }
    }
}
