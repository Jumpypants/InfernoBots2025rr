package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.murphy.MurphyTask;

@Config
public class Outtake {
    private final Motor SLIDE_MOTOR_BACK;
    private final Motor SLIDE_MOTOR_FRONT;
    private final Servo SPIN_SERVO;
    private final TouchSensor TOUCH_SENSOR;
    private final Servo CLAW_SERVO;

    public static double SLIDE_INCH_PER_ROTATION = 13.227;
    public static double ALLOWED_ERROR = 1;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double SPIN_OUT_POSITION = 0.9;
    public static double SPIN_MID_POSITION = 0.7;
    public static double SPIN_IN_POSITION = 0.47;

    public static double SPECIMEN_UP_POSITION = 25;
    public static double SPECIMEN_DOWN_POSITION = 14;

    public static double CLAW_OPEN_POSITION = 0.5;
    public static double CLAW_CLOSED_POSITION = 0.06;

    public static double HIGH_BASKET_POSITION = 42;
    public static double LOW_BASKET_POSITION = 24;
    public static double KP = 0.141;
    public static double KI = 0.01;
    public static double KD = 0.002;
    public static double KF = 0.1;

    public static double DOWN_POSITION = 0;

    private final PIDFController pidfController = new PIDFController(KP / SLIDE_INCH_PER_ROTATION, KI, KD, KF);

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR_BACK = new Motor(hardwareMap, "outtakeSlideBack");
        SLIDE_MOTOR_FRONT = new Motor(hardwareMap, "outtakeSlideFront");
        TOUCH_SENSOR = hardwareMap.get(TouchSensor.class, "touchSensor");
        SLIDE_MOTOR_BACK.setInverted(true);
        SPIN_SERVO = hardwareMap.get(Servo.class, "outtakeSpin");
        CLAW_SERVO = hardwareMap.get(Servo.class, "specimenClaw");
    }

    public void setSlideSetPoint(double setPoint) {
        if (setPoint > SLIDE_MAX_POSITION) {
            setPoint = SLIDE_MAX_POSITION;
        } else if (setPoint < SLIDE_MIN_POSITION) {
            setPoint = SLIDE_MIN_POSITION;
        }
        pidfController.setSetpoint(setPoint);
    }

    public void stepSlide(Telemetry telemetry) {
        double slidePosition = getSlidePosition(); //gets current position

        /*
        targetPosition = motion_profile_position(FILL); //gets the desired next position - determined by motion profiling
        //use this to calculate the power that the motor needs to be set to

        //MORE ADVANCED
//        targetAcceleration = motion_profile_acceleration(FILL); //gets the desired acceleration
//        targetVelocity = motion_profile_velocity(FILL); //gets the desired velocity
         */
        pidfController.setP(KP);
        pidfController.setI(KI);
        pidfController.setD(KD);
        pidfController.setF(KF);
        double power = pidfController.calculate(slidePosition);
        telemetry.addData("power", power);
        telemetry.addData("position", getSlidePosition());
        setSlidePower(power);

        if (TOUCH_SENSOR.isPressed()) {
            SLIDE_MOTOR_BACK.resetEncoder();
        }
    }

    public boolean isSlideAtSetPoint() {
        double slidePosition = getSlidePosition();
        return Math.abs(slidePosition - pidfController.getSetpoint()) < ALLOWED_ERROR;
    }

    public void setSpin(double position) {
        SPIN_SERVO.setPosition(position);
    }

    public void setClaw(double position) {
        CLAW_SERVO.setPosition(position);
    }

    public double getClawPosition () {
        return CLAW_SERVO.getPosition();
    }

    public void setSlidePower (double p) {
//        if (SLIDE_MOTOR_BACK.get() < p) {
//            SLIDE_MOTOR_BACK.set(SLIDE_MOTOR_BACK.get() + 0.003);
//        } else {
//            SLIDE_MOTOR_BACK.set(p);
//        }
//
//        if (SLIDE_MOTOR_FRONT.get() < p) {
//            SLIDE_MOTOR_FRONT.set(SLIDE_MOTOR_FRONT.get() + 0.003);
//        } else {
//            SLIDE_MOTOR_FRONT.set(p);
//        }

        SLIDE_MOTOR_BACK.set(p);
        SLIDE_MOTOR_FRONT.set(p);
    }

    public double getSlidePosition() {
        return SLIDE_MOTOR_BACK.getCurrentPosition() * SLIDE_INCH_PER_ROTATION / 360;
    }

    public Motor getSlideMotor() {
        return SLIDE_MOTOR_BACK;
    }

    public double getSlideSetPoint() {
        return pidfController.getSetpoint();
    }

    public static class MoveSlideTask extends MurphyTask {

        private final Outtake outtake;
        private final double setPoint;

        public MoveSlideTask(Outtake outtake, double setPoint) {
            this.outtake = outtake;
            this.setPoint = setPoint;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            outtake.setSlideSetPoint(setPoint);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return !outtake.isSlideAtSetPoint();
        }
    }

    public static class MoveSlideActionRR implements Action {
        private final Outtake outtake;
        private final double pos;

        public MoveSlideActionRR (Outtake outtake, double pos) {
            this.outtake = outtake;
            this.pos = pos;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            outtake.setSlideSetPoint(pos);
            return !outtake.isSlideAtSetPoint();
        }
    }


    public static class SpinToMidTask extends MurphyTask {
        private final Outtake outtake;

        public SpinToMidTask(Outtake outtake) {
            this.outtake = outtake;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            outtake.setSpin(SPIN_MID_POSITION);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return ELAPSED_TIME.seconds() < 0.5;
        }
    }

    public static class SpinToMidActionRR implements Action {
        private final Outtake outtake;
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        public SpinToMidActionRR (Outtake outtake) {
            this.outtake = outtake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                outtake.setSpin(SPIN_MID_POSITION);
            }

            return elapsedTime.seconds() < 0.5;
        }
    }

    public static class DumpTask extends MurphyTask {
        private final Outtake outtake;
        private final Gamepad gamepad2;

        private boolean buttonPressed = false;

        public DumpTask(Outtake outtake, Gamepad gamepad2) {
            this.outtake = outtake;
            this.gamepad2 = gamepad2;
        }

        @Override
        protected void initialize(Telemetry telemetry) {}

        @Override
        protected boolean run(Telemetry telemetry) {
            if (gamepad2.right_trigger > 0.5  && !buttonPressed) {
                buttonPressed = true;
                ELAPSED_TIME.reset();
                outtake.setSpin(SPIN_OUT_POSITION);
            }
            if (buttonPressed) {
                return ELAPSED_TIME.seconds() < 0.9;
            }
            return true;
        }
    }

    public static class DumpActionRR implements Action {
        private final Outtake outtake;
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        public DumpActionRR (Outtake outtake) {
            this.outtake = outtake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                outtake.setSpin(SPIN_OUT_POSITION);
            }

            return elapsedTime.seconds() < 1.1;
        }
    }

    public static class SpinToInTask extends MurphyTask {
        private final Outtake outtake;

        public SpinToInTask(Outtake outtake) {
            this.outtake = outtake;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            outtake.setSpin(SPIN_IN_POSITION);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return ELAPSED_TIME.seconds() < 0.6;
        }
    }

    public static class SpinToInActionRR implements Action {
        private final Outtake outtake;
        private final ElapsedTime elapsedTime = new ElapsedTime();
        private boolean initialized = false;

        public SpinToInActionRR (Outtake outtake) {
            this.outtake = outtake;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                initialized = true;
                elapsedTime.reset();
                outtake.setSpin(SPIN_IN_POSITION);
            }

            return elapsedTime.seconds() < 0.8;
        }
    }

    public static class ClawOpenTask extends MurphyTask {
        private final Outtake outtake;
        private double requiredTime = 0.4;

        public ClawOpenTask (Outtake outtake) {
            this.outtake = outtake;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            if (outtake.getClawPosition() == CLAW_OPEN_POSITION) {
                requiredTime = 0;
                return;
            }
            outtake.setClaw(CLAW_OPEN_POSITION);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return ELAPSED_TIME.seconds() < requiredTime;
        }
    }

    public static class ClawCloseTask extends MurphyTask {
        private final Outtake outtake;
        private double requiredTime = 0.5;

        public ClawCloseTask (Outtake outtake) {
            this.outtake = outtake;
        }

        @Override
        protected void initialize(Telemetry telemetry) {
            if (outtake.getClawPosition() == CLAW_CLOSED_POSITION) {
                requiredTime = 0;
                return;
            }
            outtake.setClaw(CLAW_CLOSED_POSITION);
        }

        @Override
        protected boolean run(Telemetry telemetry) {
            return ELAPSED_TIME.seconds() < requiredTime;
        }
    }
}
