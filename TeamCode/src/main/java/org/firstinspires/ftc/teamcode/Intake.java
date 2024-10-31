package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {
    public static enum State {
        EnterSubmersible,
        Transfer,
        ClearSamples,
        Stopped,
        Manual
    }

    public static double SLIDE_POSITION_ENTER_SUBMERSIBLE = -5;
    public static double SLIDE_POSITION_EXIT_SUBMERSIBLE = 0;
    public static double WRIST_DOWN_POSITION = 0.82;
    public static double WRIST_UP_POSITION = 0.225;
    public static double WRIST_CLEAR_POSITION = 0.5;
    public static double TRANSFER_SPIN_TIME = 2;

    public static double SLIDE_ROTATIONS_PER_INCH = 4.4 * 3;
    public static double ALLOWED_ERROR = 0.1;

    public static double KP = 0.07;
    public static double KI = 0.0;
    public static double KD = 0.0;

    public boolean finishedTransfer = true;
    public boolean finishedIntake = false;

    private final Motor SLIDE_MOTOR;
    private final SimpleServo WRIST_SERVO;
    private final Motor SPIN_MOTOR;
    private final ElapsedTime timeSpinning = new ElapsedTime();

    private double slidePosition = 0.0;
    private State state = State.Stopped;

    public Intake(HardwareMap hardwareMap) {
        this.SLIDE_MOTOR = new Motor(hardwareMap, "intakeSlide");
        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.WRIST_SERVO = new SimpleServo(hardwareMap, "intakeWrist", 0, Math.PI / 2);
        this.SPIN_MOTOR = new Motor(hardwareMap, "intakeSpin");
        SLIDE_MOTOR.setInverted(true);
    }

    public void step() {
        switch (state) {
            case EnterSubmersible:
                finishedIntake = false;
                if (stepSlideTo(SLIDE_POSITION_ENTER_SUBMERSIBLE)) {
                    setWrist(WRIST_DOWN_POSITION);
                    setState(State.Stopped);
                }
                break;
            case ClearSamples:
                if (stepSlideTo(SLIDE_POSITION_ENTER_SUBMERSIBLE)) {
                    setWrist(WRIST_CLEAR_POSITION);
                    setState(State.Stopped);
                }
                break;
            case Transfer:
                finishedIntake = true;
                if (finishedTransfer) timeSpinning.reset();
                finishedTransfer = false;
                setWrist(WRIST_UP_POSITION);
                if (stepSlideTo(SLIDE_POSITION_EXIT_SUBMERSIBLE)){
                    if (timeSpinning.seconds() < TRANSFER_SPIN_TIME) {
                        SPIN_MOTOR.set(1);
                    } else {
                        finishedTransfer = true;
                        SPIN_MOTOR.set(0);
                        setWrist(WRIST_DOWN_POSITION);
                        setState(State.Stopped);
                    }
                }
                break;
            case Stopped:
                SLIDE_MOTOR.set(0);
                break;
            case Manual:
                break;
        }
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

    public void setState (State s) {
        state = s;
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

    public Motor getSLIDE_MOTOR() {
        return SLIDE_MOTOR;
    }
}
