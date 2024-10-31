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
        TransferSampleToOuttake,
        ExtendOuttakeToBasket,
        AwaitDumpInput,
        DumpSample,
        RetractOuttakeSlideAndSpin,
    }

    private State state = State.AwaitSampleRotateInput;

    private double sampleAngle = 0;
    private double sampleDistance = 0;

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
                awaitIntakeInput(gamepad2);
                break;
            case ExtendToSample:
                extendToSample(intake, telemetry);
                break;
            case WristToSample:
                wristToSample(intake, telemetry);
                break;
            case CollectSample:
                collectSample(intake, telemetry);
                break;
            case RetractIntakeSlideAndWrist:
                retractIntakeSlideAndWrist(intake, telemetry);
                break;
            case TransferSampleToOuttake:
                transferSampleToOuttake(intake, outtake, telemetry);
                break;
            case ExtendOuttakeToBasket:
                extendOuttakeToBasket(outtake, telemetry);
                break;
            case AwaitDumpInput:
                awaitDumpInput(gamepad2);
                break;
            case DumpSample:
                dumpSample(outtake, telemetry);
                break;
            case RetractOuttakeSlideAndSpin:
                retractOuttakeSlideAndSpin(outtake, telemetry);
                break;
        }
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

    private void awaitIntakeInput(Gamepad gamepad) {
        if (gamepad.left_trigger > 0) {
            state = State.ExtendToSample;
        }
    }

    private void extendToSample(Intake intake, Telemetry telemetry) {
        if (intake.stepSlideTo(sampleDistance - Intake.EXTEND_TO_SAMPLE_OFFSET)) {
            state = State.WristToSample;
            elapsedTime.reset();
        }
    }

    private void wristToSample(Intake intake, Telemetry telemetry) {
        intake.setWrist(Intake.WRIST_DOWN_POSITION);
        if (elapsedTime.seconds() > Intake.WRIST_ROTATE_TIME) {
            state = State.CollectSample;
        }
    }

    private void collectSample(Intake intake, Telemetry telemetry) {
        intake.setSpin(-1);
        if (intake.stepSlideTo(sampleDistance)) {
            intake.setSpin(0);
            state = State.RetractIntakeSlideAndWrist;
        }
    }

    private void retractIntakeSlideAndWrist(Intake intake, Telemetry telemetry) {
        intake.setWrist(Intake.WRIST_UP_POSITION);
        if (intake.stepSlideTo(Intake.SLIDE_IN_POSITION)) {
            state = State.TransferSampleToOuttake;
            elapsedTime.reset();
        }
    }

    private void transferSampleToOuttake(Intake intake, Outtake outtake, Telemetry telemetry) {
        intake.setSpin(1);
        if (elapsedTime.seconds() > Intake.TRANSFER_SPIN_TIME) {
            intake.setSpin(0);
            state = State.ExtendOuttakeToBasket;
        }
    }

    private void extendOuttakeToBasket(Outtake outtake, Telemetry telemetry) {
        if (outtake.stepSlideTo(Outtake.HIGH_BASKET_POSITION, telemetry)) {
            state = State.AwaitDumpInput;
        }
    }

    private void awaitDumpInput(Gamepad gamepad) {
        if (gamepad.right_trigger > 0) {
            state = State.DumpSample;
            elapsedTime.reset();
        }
    }

    private void dumpSample(Outtake outtake, Telemetry telemetry) {
        outtake.setSpin(Outtake.SPIN_OUT_POSITION);
        if (elapsedTime.seconds() > Outtake.TIME_TO_SPIN) {
            outtake.setSpin(0);
            state = State.RetractOuttakeSlideAndSpin;
        }
    }

    private void retractOuttakeSlideAndSpin(Outtake outtake, Telemetry telemetry) {
        outtake.setSpin(Outtake.SPIN_IN_POSITION);
        if (outtake.stepSlideTo(Outtake.DOWN_POSITION, telemetry)) {
            state = State.AwaitSampleRotateInput;
        }
    }
}
