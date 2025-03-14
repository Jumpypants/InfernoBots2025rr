package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveBase;
import org.firstinspires.ftc.teamcode.subsystems.IntakeV0;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeV0;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

import java.util.ArrayList;

@Config
public class TeleOpStateMachineV0 {
    public enum State {
        AwaitSampleRotateInput,
        SelectSample,
        RotateToSample,
        AwaitIntakeInput,
        ExtendToSample,
        CollectSample,
        RetractIntakeSlideAndWrist,
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
            IntakeV0 intake,
            OuttakeV0 outtake,
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
                //rotateToSample(drivebase, telemetry, imu);
                break;
            case AwaitIntakeInput:
                awaitIntakeInput(gamepad2, intake);
                break;
            case ExtendToSample:
                extendToSample(intake, outtake, gamepad2, telemetry);
                break;
            case CollectSample:
                collectSample(intake, telemetry, imu, gamepad2);
                break;
            case RetractIntakeSlideAndWrist:
                retractIntakeSlideAndWrist(intake, gamepad2, telemetry);
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

    private void awaitIntakeInput(Gamepad gamepad, IntakeV0 intake) {
        intake.setWrist(IntakeV0.WRIST_UP_POSITION);
        if (gamepad.left_trigger > 0) {
            state = State.ExtendToSample;
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void extendToSample(IntakeV0 intake, OuttakeV0 outtake, Gamepad gamepad, Telemetry telemetry) {
        outtake.setSpin(OuttakeV0.SPIN_IN_POSITION);
        //intake.setWrist(Intake.WRIST_MID_POSITION);
        if (intake.stepSlideTo(IntakeV0.INITIAL_EXTENSION_DISTANCE, telemetry)) {
            state = State.CollectSample;
            intake.setWrist(IntakeV0.WRIST_MID_POSITION);
            elapsedTime.reset();
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void collectSample(IntakeV0 intake, Telemetry telemetry, IMU imu, Gamepad gamepad) {
        intake.stepSlideTo(intake.getSlidePosition() - gamepad.left_stick_y * 2, telemetry);

        if (gamepad.right_bumper) {
            intake.setSpin(IntakeV0.SPIN_IN);
        }
        if (gamepad.left_bumper) {
            intake.setSpin(IntakeV0.SPIN_OUT);
        }

        if (gamepad.right_stick_y < -0.2 || gamepad.a) {
            intake.setWrist(IntakeV0.WRIST_DOWN_POSITION);
            intake.setSpin(IntakeV0.SPIN_IN);
        }
        if (gamepad.right_stick_y > 0.2 || gamepad.b) {
            intake.setWrist(IntakeV0.WRIST_MID_POSITION);
        }

        if (gamepad.right_trigger > 0.1 || intake.getColor(telemetry) == Color.YELLOW) {
            state = State.RetractIntakeSlideAndWrist;
            elapsedTime.reset();
            intake.setSpin(IntakeV0.SPIN_STOP);
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void retractIntakeSlideAndWrist(IntakeV0 intake, Gamepad gamepad, Telemetry telemetry) {
        intake.setWrist(IntakeV0.WRIST_UP_POSITION);
        if (intake.stepSlideTo(IntakeV0.SLIDE_IN_POSITION, telemetry) && elapsedTime.seconds() > 1.2) {
            state = State.AwaitTransferInput;
        }

        if (gamepad.x) {
            state = State.IntakeManualControl;
        }
    }

    private void awaitTransferInput (Gamepad gamepad2, OuttakeV0 outtake, IntakeV0 intake, Telemetry telemetry) {
        intake.setWrist(IntakeV0.WRIST_UP_POSITION);
        if (gamepad2.right_trigger > 0) {
            state = State.TransferSampleToOuttake;
            elapsedTime.reset();
            basketPos = OuttakeV0.HIGH_BASKET_POSITION;
        }

        if (gamepad2.right_bumper)  {
            state = State.TransferSampleToOuttake;
            elapsedTime.reset();
            basketPos = OuttakeV0.LOW_BASKET_POSITION;
        }

        if (gamepad2.left_trigger > 0) {
            state = State.ExtendToSample;
        }
    }

    private void transferSampleToOuttake(IntakeV0 intake, OuttakeV0 outtake, Telemetry telemetry) {
        intake.setSpin(IntakeV0.SPIN_OUT);
        if (elapsedTime.seconds() > IntakeV0.TRANSFER_SPIN_TIME) {
            elapsedTime.reset();
            state = State.GetWristOutOfWay;
            intake.setSpin(IntakeV0.SPIN_STOP);
        }
    }

    private void getWristOutOfWay(IntakeV0 intake) {
        intake.setWrist(IntakeV0.WRIST_MID_POSITION);
        if (elapsedTime.seconds() > 0.5) {
            state = State.ExtendOuttakeToBasket;
        }
    }

    private void extendOuttakeToBasket(OuttakeV0 outtake, Gamepad gamepad, Telemetry telemetry) {
        if (outtake.stepSlideTo(basketPos, telemetry)) {
            state = State.AwaitDumpInput;
        }

        if (gamepad.x) {
            state = State.OuttakeManualControl;
        }

        telemetry.addData("Outtake pos", outtake.getSlidePosition());
        telemetry.addData("outtake target", OuttakeV0.HIGH_BASKET_POSITION);
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

    private void dumpSample(OuttakeV0 outtake, Gamepad gamepad, Telemetry telemetry) {
        outtake.setSpin(OuttakeV0.SPIN_OUT_POSITION);
        if (elapsedTime.seconds() > OuttakeV0.TIME_TO_SPIN) {
            state = State.RetractOuttakeSlideAndSpin;
        }

        if (gamepad.x) {
            state = State.OuttakeManualControl;
        }
    }

    private void retractOuttakeSlideAndSpin(OuttakeV0 outtake, Gamepad gamepad, Telemetry telemetry) {
        outtake.setSpin(OuttakeV0.SPIN_IN_POSITION);
        if (outtake.stepSlideTo(OuttakeV0.DOWN_POSITION, telemetry)) {
            outtake.getSlideMotor().set(0);
            state = State.AwaitIntakeInput;
        }

        if (gamepad.x) {
            state = State.OuttakeManualControl;
        }

        if (gamepad.left_trigger > 0.1) {
            //state = State.RetractOuttakeExtendIntake;
        }
    }

    private void retractOuttakeExtendIntake(OuttakeV0 outtake, IntakeV0 intake, Gamepad gamepad2, Telemetry telemetry) {
        intake.setWrist(IntakeV0.WRIST_MID_POSITION);
        if (outtake.stepSlideTo(OuttakeV0.DOWN_POSITION, telemetry)) {
            state = State.ExtendToSample;
            outtake.getSlideMotor().set(0);
        }
        if (intake.stepSlideTo(IntakeV0.INITIAL_EXTENSION_DISTANCE, telemetry)) {
            intake.getSlideMotor().set(0);
        }
    }

    private void outtakeManualControl(OuttakeV0 outtake, Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.left_bumper) {
            outtake.setSpin(OuttakeV0.SPIN_OUT_POSITION);
        }

        if (gamepad.right_bumper) {
            outtake.setSpin(OuttakeV0.SPIN_IN_POSITION);
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

    private void intakeManualControl(IntakeV0 intake, Gamepad gamepad, Telemetry telemetry) {
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
//            intake.setWrist(Intake.WRIST_TRANSFER_POSITION);
//        }
//
//        if (gamepad.y) {
//            state = State.RetractIntakeSlideAndWrist;
//        }
//
//        intake.stepSlideTo(intake.getSlidePosition() - gamepad.left_stick_y * 2, telemetry);
    }
}
