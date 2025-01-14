package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.murphy.MurphyAction;

@Config
public class Intake {
    public static double SAMPLE_DISTANCE = 1.5;

    public static double WRIST_DOWN_POSITION = 0.235;
    public static double WRIST_MID_POSITION = 0.4;
    public static double WRIST_UP_POSITION = 0.79;

    public static double WRIST_EXTEND_INCREASE = 0.02;

    public static double SPIN_IN = -1;
    public static double SPIN_OUT = 1;
    public static double SPIN_STOP = 0;

    public static double SLIDE_IN_POSITION = 0;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double INITIAL_EXTENSION_DISTANCE = 8;
    public static double SLIDE_TICKS_PER_INCH = 0.0454545;
    public static double ALLOWED_ERROR = 1;


    public static double KP = 0.015;
    public static double KI = 0.0;
    public static double KD = 0.01;

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

    public void stepSlide(double powerCoefficient, Telemetry telemetry) {
        double slidePosition = getSlidePosition();
        double power = pidController.calculate(slidePosition);
        SLIDE_MOTOR.set(power * powerCoefficient);


        if (getSlidePosition() > 22 && getWrist() == WRIST_DOWN_POSITION) {
            setWrist(WRIST_DOWN_POSITION + WRIST_EXTEND_INCREASE);
        } else if (getSlidePosition() < 22 && getWrist() == WRIST_DOWN_POSITION + WRIST_EXTEND_INCREASE) {
            setWrist(WRIST_DOWN_POSITION);
        }

        telemetry.addData("Red", COLOR_SENSOR.red());
        telemetry.addData("Green", COLOR_SENSOR.green());
        telemetry.addData("Blue", COLOR_SENSOR.blue());
        telemetry.addData("Distance", DISTANCE_SENSOR.getDistance(DistanceUnit.INCH));
    }

    public void stepSlide(Telemetry telemetry) {
        stepSlide(1, telemetry);
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

    public double getSlideSetPoint() {return pidController.getSetpoint();}

    public void driveSlide(double x, double y, double heading, boolean slowMode) {
        double cosHeading = Math.cos(Math.toRadians(heading));
        double sinHeading = Math.sin(Math.toRadians(heading));

        double power = x * sinHeading + y * cosHeading;

        pidController.setSetpoint(getSlidePosition() - y * (slowMode ? 2 : 8));
    }

    public boolean hasYellowSample() {
        return COLOR_SENSOR.red() >= 240 && COLOR_SENSOR.green() >= 294 && COLOR_SENSOR.blue() <= 300;
    }

    public boolean hasBlueSample() {
        return COLOR_SENSOR.red() <= 110 && COLOR_SENSOR.green() <= 220 && COLOR_SENSOR.blue() >= 130;
    }

    public boolean hasRedSample() {
        return COLOR_SENSOR.red() >= 150 && COLOR_SENSOR.green() <= 293 && COLOR_SENSOR.blue() <= 120;
    }

    public boolean hasSampleOfCorrectColor(Robot.Alliance alliance) {
        if (DISTANCE_SENSOR.getDistance(DistanceUnit.INCH) > SAMPLE_DISTANCE) return false;
        if (hasYellowSample()) return true;
        if (alliance == Robot.Alliance.RED) {
            return hasRedSample();
        } else {
            return hasBlueSample();
        }

    }

    public boolean hasSample() {
        return DISTANCE_SENSOR.getDistance(DistanceUnit.INCH) <= SAMPLE_DISTANCE;
    }


    public static class WristAction extends MurphyAction {
        private final double requiredTime;
        private final Intake intake;
        private final double wristPosition;

        public WristAction(Intake intake, double wristPosition) {
            requiredTime = Math.abs(wristPosition - intake.getWrist()) * 1.5;
            this.intake = intake;
            this.wristPosition = wristPosition;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            intake.setWrist(wristPosition);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return ELAPSED_TIME.seconds() < requiredTime;
        }
    }

    public static class MoveSlideAction extends MurphyAction {
        private final Intake intake;
        private final double setPoint;

        public MoveSlideAction(Intake intake, double setPoint) {
            this.intake = intake;
            this.setPoint = setPoint;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            intake.setSlideSetPoint(setPoint);}

        @Override
        protected boolean run(Telemetry telemetry) {
            return !intake.isSlideAtSetPoint();
        }
    }

    public static class TransferSpinAction extends MurphyAction {
        private final Intake intake;

        public TransferSpinAction(Intake intake) {
            this.intake = intake;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            intake.setSpin(SPIN_OUT);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            if (ELAPSED_TIME.seconds() > 0.45) {
                intake.setSpin(SPIN_STOP);
                return false;
            }
            return true;
        }
    }

    public static class ClearWristAction extends MurphyAction {
        private final Intake intake;

        public ClearWristAction(Intake intake) {
            this.intake = intake;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            intake.setWrist(WRIST_MID_POSITION);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return ELAPSED_TIME.seconds() < 0.4;
        }
    }
}
