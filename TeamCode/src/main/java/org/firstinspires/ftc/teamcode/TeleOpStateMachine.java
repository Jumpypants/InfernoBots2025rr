package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

@Config
public class TeleOpStateMachine {
    public enum State {
        AwaitSampleRotateInput,
        SelectSample,
        RotateToSample,
        AwaitIntakeInput,
        ExtendToSample,
        WristToSample,
        CollectSample,
        RetractIntakeSlideAndWrist,
        AwaitWristUpInput,
        WristToOuttake,
        AwaitTransferInput,
        TransferSampleToOuttake,
        GetWristOutOfWay,
        ExtendOuttakeToBasket,
        AwaitDumpInput,
        DumpSample,
        RetractOuttakeSlideAndSpin,
        RetractOuttakeExtendIntake,
        OuttakeManualControl,
        IntakeManualControl,
    }

    private State state = State.AwaitIntakeInput;

    private double sampleAngle = 0;
    private double sampleDistance = 7;
    private double basketPos = 0;

    private final ElapsedTime elapsedTime = new ElapsedTime();

    public void step(
            Intake intake,
            Outtake outtake,
            DriveBase drivebase,
            Telemetry telemetry,
            IMU imu,
            Gamepad gamepad1,
            Gamepad gamepad2,
            SampleFinder sampleFinder
    ) {
        telemetry.addData("State", state);

        switch (state) {
            case AwaitSampleRotateInput:
                awaitSampleRotateInput(gamepad1, telemetry);
                break;
            case SelectSample:
                selectSample(sampleFinder, telemetry, imu);
                break;
            case RotateToSample:
                rotateToSample(drivebase, telemetry, imu);
                break;
            case AwaitIntakeInput:
                awaitIntakeInput(gamepad2, intake);
                break;
            case ExtendToSample:
                extendToSample(intake, outtake, gamepad2, telemetry);
                break;
            case WristToSample:
                wristToSample(intake, gamepad2, telemetry);
                break;
            case CollectSample:
                collectSample(intake, drivebase, telemetry, gamepad2);
                break;
            case RetractIntakeSlideAndWrist:
                retractIntakeSlideAndWrist(intake, gamepad2, telemetry);
                break;
            case AwaitWristUpInput:
                awaitWristUpInput(intake, gamepad2, telemetry);
                break;
            case WristToOuttake:
                wristToOuttake(intake, telemetry);
                break;
            case AwaitTransferInput:
                awaitTransferInput(gamepad2, outtake, intake, telemetry);
                break;
            case TransferSampleToOuttake:
                transferSampleToOuttake(intake, outtake, telemetry);
                break;
            case GetWristOutOfWay:
                getWristOutOfWay(intake);
                break;
            case ExtendOuttakeToBasket:
                extendOuttakeToBasket(outtake, gamepad2, telemetry);
                break;
            case AwaitDumpInput:
                awaitDumpInput(gamepad2);
                break;
            case DumpSample:
                dumpSample(outtake, gamepad2, telemetry);
                break;
            case RetractOuttakeSlideAndSpin:
                retractOuttakeSlideAndSpin(outtake, gamepad2, telemetry);
                break;
            case RetractOuttakeExtendIntake:
                retractOuttakeExtendIntake(outtake, intake, gamepad2, telemetry);
                break;

            case OuttakeManualControl:
                outtakeManualControl(outtake, gamepad2, telemetry);
                break;
            case IntakeManualControl:
                intakeManualControl(intake, gamepad2, telemetry);
                break;
        }
    }

    public void setState(State state) {
        this.state = state;
    }

    private void awaitSampleRotateInput(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.a) {
            state = State.SelectSample;
        }
    }

    private void selectSample(SampleFinder sampleFinder, Telemetry telemetry, IMU imu) {
        // TODO: Implement sample selection
        // This is a placeholder
        ArrayList<Sample> samples = sampleFinder.get(telemetry);
        sampleAngle = imu.getRobotYawPitchRollAngles().getYaw() + samples.get(0).getAngle();
        sampleDistance = samples.get(0).getDistance();
        state = State.RotateToSample;
    }

    private void rotateToSample(DriveBase drivebase, Telemetry telemetry, IMU imu) {
        if (drivebase.stepRotateTo(
                sampleAngle,
                imu.getRobotYawPitchRollAngles().getYaw(),
                telemetry,
                1 / sampleDistance
        )) {
            state = State.AwaitIntakeInput;
        }
    }

    private void awaitIntakeInput(Gamepad gamepad, Intake intake) {
        intake.setWrist(Intake.WRIST_MID_POSITION);
        if (gamepad.left_trigger > 0) {
            state = State.ExtendToSample;
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void extendToSample(Intake intake, Outtake outtake, Gamepad gamepad, Telemetry telemetry) {
        outtake.setSpin(Outtake.SPIN_IN_POSITION);
        intake.setWrist(Intake.WRIST_MID_POSITION);
        if (intake.stepSlideTo(sampleDistance + Intake.EXTEND_TO_SAMPLE_OFFSET, telemetry)) {
            state = State.WristToSample;
            elapsedTime.reset();
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void wristToSample(Intake intake, Gamepad gamepad, Telemetry telemetry) {
        //intake.setWrist(Intake.WRIST_DOWN_POSITION);
        if (elapsedTime.seconds() > Intake.WRIST_ROTATE_TIME) {
            state = State.CollectSample;
            elapsedTime.reset();
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void collectSample(Intake intake, DriveBase driveBase, Telemetry telemetry, Gamepad gamepad) {

        if (gamepad.left_trigger > 0.1) {
            intake.stepSlideTo(intake.getSlidePosition() - gamepad.left_stick_y, telemetry);
        } else {
            intake.stepSlideTo(intake.getSlidePosition() - 2 * (gamepad.left_stick_y), telemetry);
        }

        //driveBase.rotate(-gamepad.left_stick_x / 1.5);

        if (gamepad.right_bumper) {
            intake.setClaw(Intake.CLAW_CLOSED_POSITION);
        }
        if (gamepad.left_bumper) {
            intake.setClaw(Intake.CLAW_OPEN_POSITION);
        }

        if (gamepad.a) {
            intake.setWrist(Intake.WRIST_DOWN_POSITION);
        }
        if (gamepad.b) {
            intake.setWrist(Intake.WRIST_MID_POSITION);
        }

        if (gamepad.right_trigger > 0) {
            state = State.RetractIntakeSlideAndWrist;
            elapsedTime.reset();
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void retractIntakeSlideAndWrist(Intake intake, Gamepad gamepad, Telemetry telemetry) {
        intake.setWrist(Intake.WRIST_MID_POSITION);
        if (intake.stepSlideTo(Intake.SLIDE_IN_POSITION, telemetry) && elapsedTime.seconds() > 1.2) {
            state = State.AwaitWristUpInput;
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void awaitWristUpInput(Intake intake, Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.left_trigger > 0) {
            state = State.ExtendToSample;
        }

        if (gamepad.right_trigger > 0) {
            elapsedTime.reset();
            state = State.WristToOuttake;
        }
    }

    private void wristToOuttake(Intake intake, Telemetry telemetry) {
        intake.setWrist(Intake.WRIST_UP_POSITION);
        if (elapsedTime.seconds() > 1.5) {
            state = State.AwaitTransferInput;
        }
    }

    private void awaitTransferInput (Gamepad gamepad2, Outtake outtake, Intake intake, Telemetry telemetry) {
        intake.setWrist(Intake.WRIST_UP_POSITION);
        if (gamepad2.right_trigger > 0) {
            state = State.TransferSampleToOuttake;
            elapsedTime.reset();
            basketPos = Outtake.HIGH_BASKET_POSITION;
        }

        if (gamepad2.right_bumper)  {
            state = State.TransferSampleToOuttake;
            elapsedTime.reset();
            basketPos = Outtake.LOW_BASKET_POSITION;
        }

        if (gamepad2.left_trigger > 0) {
            state = State.ExtendToSample;
        }
    }

    private void transferSampleToOuttake(Intake intake, Outtake outtake, Telemetry telemetry) {
        intake.setClaw(Intake.CLAW_OPEN_POSITION);
        if (elapsedTime.seconds() > Intake.TRANSFER_TIME) {
            elapsedTime.reset();
            state = State.GetWristOutOfWay;
        }
    }

    private void getWristOutOfWay(Intake intake) {
        intake.setWrist(Intake.WRIST_MID_POSITION);
        if (elapsedTime.seconds() > 0.5) {
            state = State.ExtendOuttakeToBasket;
        }
    }

    private void extendOuttakeToBasket(Outtake outtake, Gamepad gamepad, Telemetry telemetry) {
        if (outtake.stepSlideTo(basketPos, telemetry)) {
            state = State.AwaitDumpInput;
        }

        if (gamepad.x) {
            state = State.OuttakeManualControl;
        }

        telemetry.addData("Outtake pos", outtake.getSlidePosition());
        telemetry.addData("outtake target", Outtake.HIGH_BASKET_POSITION);
    }

    private void awaitDumpInput(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            state = State.DumpSample;
            elapsedTime.reset();
        }

        if (gamepad.x) {
            state = State.OuttakeManualControl;
        }
    }

    private void dumpSample(Outtake outtake, Gamepad gamepad, Telemetry telemetry) {
        outtake.setSpin(Outtake.SPIN_OUT_POSITION);
        if (elapsedTime.seconds() > Outtake.TIME_TO_SPIN) {
            state = State.RetractOuttakeSlideAndSpin;
        }

        if (gamepad.x) {
            state = State.OuttakeManualControl;
        }
    }

    private void retractOuttakeSlideAndSpin(Outtake outtake, Gamepad gamepad, Telemetry telemetry) {
        outtake.setSpin(Outtake.SPIN_IN_POSITION);
        if (outtake.stepSlideTo(Outtake.DOWN_POSITION, telemetry)) {
            outtake.getSlideMotor().set(0);
            state = State.AwaitIntakeInput;
        }

        if (gamepad.x) {
            state = State.OuttakeManualControl;
        }

//        if (gamepad.left_trigger > 0) {
//            state = State.RetractOuttakeExtendIntake;
//        }
    }

    private void retractOuttakeExtendIntake(Outtake outtake, Intake intake, Gamepad gamepad2, Telemetry telemetry) {
        intake.setWrist(Intake.WRIST_MID_POSITION);
        if (outtake.stepSlideTo(Outtake.DOWN_POSITION, telemetry)) {
            state = State.ExtendToSample;
        }
        if (intake.stepSlideTo(sampleDistance + Intake.EXTEND_TO_SAMPLE_OFFSET, telemetry)) {
            intake.getSlideMotor().set(0);
        }
    }

    private void outtakeManualControl(Outtake outtake, Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.left_bumper) {
            outtake.setSpin(Outtake.SPIN_OUT_POSITION);
        }

        if (gamepad.right_bumper) {
            outtake.setSpin(Outtake.SPIN_IN_POSITION);
        }

        if (gamepad.y) {
            state = State.RetractOuttakeSlideAndSpin;
        }

        if (gamepad.left_trigger > 0.1) {
            outtake.stepSlideTo(outtake.getSlidePosition() - gamepad.left_stick_y, telemetry);
        } else {
            outtake.stepSlideTo(outtake.getSlidePosition() - gamepad.left_stick_y * 2, telemetry);
        }
    }

    private void intakeManualControl(Intake intake, Gamepad gamepad, Telemetry telemetry) {
//        if (gamepad.right_bumper) {
//            intake.getSpinServo().set(1);
//        }
//
//        if (gamepad.left_bumper) {
//            intake.getSpinServo().set(-1);
//        }
//
//        if (gamepad.a) {
//            intake.getSpinServo().set(0);
//        }
//
//        if (gamepad.right_trigger > 0) {
//            intake.setWrist(Intake.WRIST_DOWN_POSITION);
//        }
//
//        if (gamepad.left_trigger > 0) {
//            intake.setWrist(Intake.WRIST_UP_POSITION);
//        }
//
//        if (gamepad.y) {
//            state = State.RetractIntakeSlideAndWrist;
//        }
//
//        intake.stepSlideTo(intake.getSlidePosition() - gamepad.left_stick_y * 2, telemetry);
    }
}
