package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PIDController;
import org.firstinspires.ftc.teamcode.murphy.MurphyTask;

@Config
public class Intake {
    public static double TOP_DOWN_WRIST_OFFSET = -0.04;
    public static double SAMPLE_DISTANCE = 0.7;

    public static double WRIST_DOWN_POSITION = 0.21;
    public static double WRIST_MID_POSITION = 0.3;
    public static double WRIST_TRANSFER_POSITION = 0.85;
    public static double WRIST_UP_POSITION = 0.6;

    public static double FLIP_LOW_POSITION = 0.125;
    public static double FLIP_HIGH_POSITION = 0.8;

    public static double SPIN_IN = -1;
    public static double SPIN_OUT = 1;
    public static double SPIN_STOP = 0;

    public static double SLIDE_IN_POSITION = 0;

    public static double SLIDE_MAX_POSITION = 18.5;
    public static double SLIDE_MIN_POSITION = 0;

    public static double SLIDE_TICKS_PER_INCH = 0.0454545;
    public static double ALLOWED_ERROR = 1;

    public static double KP = 0.015;
    public static double KI = 0.0;
    public static double KD = 0.01;

    private final PIDController pidController = new PIDController(KP / SLIDE_TICKS_PER_INCH, KI, KD);

    private final Motor SLIDE_MOTOR;

    private final Servo WRIST_LEFT_SERVO;
    private final Servo WRIST_RIGHT_SERVO;

    private final Servo FLIP_SERVO;

    private final CRServo RIGHT_SPIN_SERVO;
    private final CRServo LEFT_SPIN_SERVO;

    private final ColorSensor COLOR_SENSOR;
    private final DistanceSensor DISTANCE_SENSOR;

    private boolean isFlippedLow = false;

    public Intake(HardwareMap hardwareMap) {
        SLIDE_MOTOR = new Motor(hardwareMap, "intakeSlide");
        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        SLIDE_MOTOR.setInverted(true);

        WRIST_LEFT_SERVO = hardwareMap.get(Servo.class, "intakeWristLeft");
        WRIST_RIGHT_SERVO = hardwareMap.get(Servo.class, "intakeWristRight");

        RIGHT_SPIN_SERVO = new CRServo(hardwareMap, "intakeSpinRight");
        RIGHT_SPIN_SERVO.setRunMode(CRServo.RunMode.RawPower);

        LEFT_SPIN_SERVO = new CRServo(hardwareMap, "intakeSpinLeft");
        LEFT_SPIN_SERVO.setRunMode(CRServo.RunMode.RawPower);

        FLIP_SERVO = hardwareMap.get(Servo.class, "intakeFlip");

        COLOR_SENSOR = hardwareMap.get(ColorSensor.class, "colorSensor");
        DISTANCE_SENSOR = hardwareMap.get(DistanceSensor.class, "colorSensor");
    }

    public void setSlideSetPoint(double setPoint) {
        if (setPoint > SLIDE_MAX_POSITION) {
            setPoint = SLIDE_MAX_POSITION;
        }
        pidController.setSetpoint(setPoint);
    }

    public void stepSlide(double powerCoefficient, Telemetry telemetry) {
        double slidePosition = getSlidePosition();
        double power = pidController.calculate(slidePosition);
        SLIDE_MOTOR.set(power * powerCoefficient);
        telemetry.addData("Intake pos", getSlidePosition());
        telemetry.addData("intake setpoint", pidController.getSetpoint());
    }

    public void stepSlide(Telemetry telemetry) {
        stepSlide(1, telemetry);
    }

    public boolean isSlideAtSetPoint() {
        double slidePosition = getSlidePosition();
        return Math.abs(pidController.getSetpoint() - slidePosition) < ALLOWED_ERROR;
    }

    public void setSpin (double p) {
        RIGHT_SPIN_SERVO.set(p);
        LEFT_SPIN_SERVO.set(-p);
    }

    public double getSpin() {
        return RIGHT_SPIN_SERVO.get();
    }

    public void setWrist (double p) {
        WRIST_LEFT_SERVO.setPosition(p);
        WRIST_RIGHT_SERVO.setPosition(1 - p);
    }

    public void setFlip (double p) {
        isFlippedLow = p == FLIP_LOW_POSITION;
        FLIP_SERVO.setPosition(p);
    }

    public boolean getIsFlippedLow () {
        return isFlippedLow;
    }

    public double getSlidePosition() {
        return (double) (SLIDE_MOTOR.getCurrentPosition()) * SLIDE_TICKS_PER_INCH;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR;
    }

    public double getSlideSetPoint() {return pidController.getSetpoint();}

    public void driveSlide(double x, double y, double heading, boolean slowMode) {
        setSlideSetPoint(getSlidePosition() - y * (slowMode ? 2 : 8));
    }

    public boolean hasYellowSample() {
        return COLOR_SENSOR.green() > COLOR_SENSOR.red() && COLOR_SENSOR.green() > COLOR_SENSOR.blue();
    }

    public boolean hasBlueSample() {
        return COLOR_SENSOR.blue() > COLOR_SENSOR.red() && COLOR_SENSOR.blue() > COLOR_SENSOR.green();
    }

    public boolean hasRedSample() {
        return COLOR_SENSOR.red() > COLOR_SENSOR.green() && COLOR_SENSOR.red() > COLOR_SENSOR.blue();
    }

    public boolean hasSampleOfCorrectColor(Robot.Alliance alliance) {
        if (!hasSample()) return false;
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

    public static class FlipTask extends MurphyTask {
        private final double requiredTime;
        private final Intake intake;
        private final double flipPosition;

        public FlipTask(Intake intake, double flipPosition) {
            requiredTime = 0.5;
            this.intake = intake;
            this.flipPosition = flipPosition;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            intake.setFlip(flipPosition);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return ELAPSED_TIME.seconds() < requiredTime;
        }
    }

    public static class FlipActionRR implements Action {
        private final double requiredTime;
        private final Intake intake;
        private final double position;
        private boolean initialized = false;
        private final ElapsedTime elapsedTime = new ElapsedTime();

        public FlipActionRR(Intake intake, double position) {
            requiredTime = 0.5;
            this.intake = intake;
            this.position = position;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                intake.setFlip(position);
            }

            return elapsedTime.seconds() < requiredTime;
        }
    }

    public static class WristTask extends MurphyTask {
        private final double requiredTime;
        private final Intake intake;
        private final double wristPosition;

        public WristTask(Intake intake, double wristPosition) {
            requiredTime = 0.7;
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

    public static class WristActionRR implements Action {
        private final double requiredTime;
        private final Intake intake;
        private final double wristPosition;
        private boolean initialized = false;
        private final ElapsedTime elapsedTime = new ElapsedTime();

        public WristActionRR(Intake intake, double wristPosition) {
            requiredTime = 1.2;
            this.intake = intake;
            this.wristPosition = wristPosition;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                intake.setWrist(wristPosition);
            }

            return elapsedTime.seconds() < requiredTime;
        }
    }

    public static class MoveSlideTask extends MurphyTask {
        private final Intake intake;
        private final double setPoint;

        public MoveSlideTask(Intake intake, double setPoint) {
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

    public static class MoveSlideActionRR implements Action {
        private final Intake intake;
        private final double setPoint;

        public MoveSlideActionRR(Intake intake, double setPoint) {
            this.intake = intake;
            this.setPoint = setPoint;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intake.setSlideSetPoint(setPoint);
            telemetryPacket.put("Slide", "moving");
            return !intake.isSlideAtSetPoint();
        }
    }

    public static class TransferSpinTask extends MurphyTask {
        private final Intake intake;

        public TransferSpinTask(Intake intake) {
            this.intake = intake;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            intake.setSpin(SPIN_OUT);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            if (ELAPSED_TIME.seconds() > 0.4) {
                intake.setSpin(SPIN_STOP);
                return false;
            }
            return true;
        }
    }

    public static class TransferSpinActionRR implements Action {
        private final Intake intake;
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        public TransferSpinActionRR(Intake intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                intake.setSpin(Intake.SPIN_OUT);
                elapsedTime.reset();
            }

            if (elapsedTime.seconds() > 0.2) {
                intake.setSpin(SPIN_STOP);
                return false;
            }

            return true;
        }
    }

    public static class ClearWristTask extends MurphyTask {
        private final Intake intake;

        public ClearWristTask(Intake intake) {
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

    public static class ClearWristActionRR implements Action {
        private final Intake intake;
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        public ClearWristActionRR(Intake intake) {
            this.intake = intake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                intake.setWrist(WRIST_MID_POSITION);
            }

            return elapsedTime.seconds() < 0.4;
        }
    }

    public static class CollectSampleHighActionRR implements Action {
        private final Intake intake;
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private final double distance;

        private boolean initialized = false;

        public CollectSampleHighActionRR(Intake intake, double distance) {
            this.intake = intake;
            this.distance = distance;
        }

        public CollectSampleHighActionRR(Intake intake) {
            this.intake = intake;
            distance = -1;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                if (distance > 0) intake.setSlideSetPoint(distance);
            }

            if (elapsedTime.seconds() > 0.75) {
                intake.setSpin(Intake.SPIN_STOP);
                return false;
            }

            if (intake.isSlideAtSetPoint() || distance < 0) {
                intake.setWrist(WRIST_DOWN_POSITION + TOP_DOWN_WRIST_OFFSET);
                intake.setSpin(SPIN_IN);
                if (intake.hasSample()) {
                    intake.setSpin(SPIN_STOP);
                    return false;
                }
            }

            return true;
        }
    }

    public static class CollectSampleLowActionRR implements Action {
        private final Intake intake;
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private final Robot.Alliance ALLIANCE;

        private boolean initialized = false;

        public CollectSampleLowActionRR(Intake intake, Robot.Alliance alliance) {
            this.intake = intake;
            this.ALLIANCE = alliance;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                intake.setSpin(SPIN_IN);
            }

            intake.setSlideSetPoint(intake.getSlidePosition() + 2.5);

            if (intake.hasSampleOfCorrectColor(ALLIANCE) || elapsedTime.seconds() > 2) {
                intake.setSpin(SPIN_STOP);
                return false;
            }

            if (intake.hasSample() && !intake.hasSampleOfCorrectColor(ALLIANCE)){
                intake.setSpin(SPIN_OUT);
                return false;
            }

            return true;
        }
    }
}
