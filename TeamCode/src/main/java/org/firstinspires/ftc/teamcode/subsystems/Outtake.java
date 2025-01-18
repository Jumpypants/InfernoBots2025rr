package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PIDFController;
import org.firstinspires.ftc.teamcode.murphy.MurphyAction;

@Config
public class Outtake {
    private final Motor SLIDE_MOTOR_BACK;
    private final Motor SLIDE_MOTOR_FRONT;
    private final Servo SPIN_SERVO;

    public static double SLIDE_INCH_PER_ROTATION = 4.724;
    public static double ALLOWED_ERROR = 2;

    public static double SLIDE_MAX_POSITION = 42;
    public static double SLIDE_MIN_POSITION = 0;

    public static double SPIN_OUT_POSITION = 1;
    public static double SPIN_MID_POSITION = 0.7;
    public static double SPIN_IN_POSITION = 0.5;

    public static double HIGH_BASKET_POSITION = 41;
    public static double LOW_BASKET_POSITION = 24;
    public static double DOWN_POSITION = 0.0;

    public static double KP = 0.02;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.1;

    private static final int slideLength = 42;

    private final PIDFController pidfController = new PIDFController(KP / SLIDE_INCH_PER_ROTATION, KI, KD, KF);

    public Outtake(HardwareMap hardwareMap) {
        SLIDE_MOTOR_BACK = new Motor(hardwareMap, "outtakeSlideBack");
        SLIDE_MOTOR_FRONT = new Motor(hardwareMap, "outtakeSlideFront");
        //SLIDE_MOTOR_BACK.setInverted(true);
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
    }

    public boolean isSlideAtSetPoint() {
        double slidePosition = getSlidePosition();
        return Math.abs(slidePosition - pidfController.getSetpoint()) < ALLOWED_ERROR;
    }

    public void setSpin(double position) {
        SPIN_SERVO.setPosition(position);
    }

    public void setSlidePower (double p) {
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

    double motion_profile(double max_acceleration, double max_velocity, double distance, double elapsed_time) {
        /*
        Return the current reference position based on the given motion profile times, maximum acceleration, velocity, and current time.
        */

        // Calculate the time it takes to accelerate to max velocity
        double acceleration_dt = max_velocity / max_acceleration;

        // If we can't accelerate to max velocity in the given distance, we'll accelerate as much as possible
        double halfway_distance = distance / 2;
        double acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        if (acceleration_distance > halfway_distance) {
            acceleration_dt = Math.sqrt(halfway_distance / (0.5 * max_acceleration));
        }

        acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);

        // recalculate max velocity based on the time we have to accelerate and decelerate
        max_velocity = max_acceleration * acceleration_dt;

        // we decelerate at the same rate as we accelerate
        double deceleration_dt = acceleration_dt;

        // calculate the time that we're at max velocity
        double cruise_distance = distance - 2 * acceleration_distance;
        double cruise_dt = cruise_distance / max_velocity;
        double deceleration_time = acceleration_dt + cruise_dt;

        // check if we're still in the motion profile
        double entire_dt = acceleration_dt + cruise_dt + deceleration_dt;
        if (elapsed_time > entire_dt) {
            return distance;
        }

        // if we're accelerating
        if (elapsed_time < acceleration_dt) {
            // use the kinematic equation for acceleration
            return 0.5 * max_acceleration *  Math.pow(elapsed_time, 2);
        }

        // if we're cruising
        else if (elapsed_time < deceleration_time) {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            double cruise_current_dt = elapsed_time - acceleration_dt;

            // use the kinematic equation for constant velocity
            return acceleration_distance + max_velocity * cruise_current_dt;
        }

        // if we're decelerating
        else {
            acceleration_distance = 0.5 * max_acceleration * Math.pow(acceleration_dt, 2);
            cruise_distance = max_velocity * cruise_dt;
            deceleration_time = elapsed_time - deceleration_time;

            // use the kinematic equations to calculate the instantaneous desired position
            return acceleration_distance + cruise_distance + max_velocity * deceleration_time - 0.5 * max_acceleration * Math.pow(deceleration_time, 2);
        }
    }

    public static class MoveSlideAction extends MurphyAction {
        private final Outtake outtake;
        private final double setPoint;

        public MoveSlideAction(Outtake outtake, double setPoint) {
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


    public static class SpinToMidAction extends MurphyAction {
        private final Outtake outtake;

        public SpinToMidAction(Outtake outtake) {
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

    public static class DumpAction extends MurphyAction {
        private final Outtake outtake;
        private final Gamepad gamepad2;

        private boolean buttonPressed = false;

        public DumpAction(Outtake outtake, Gamepad gamepad2) {
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
//    public double trapezoid (double maxVelocity, double maxAcceleration, double inputTime){
//        double constantTime = totalTime - (2*accelerationTime);
//        double height = ((totalTime + constantTime)/2) * 42;
//        double velocity =
//    }
}
