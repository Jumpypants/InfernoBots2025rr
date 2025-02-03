package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.murphy.MurphyTask;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakingState implements MurphyState {
    private final Robot robot;

    private MurphyTask wristMoveTask = null;

    public IntakingState(Robot robot) {
        this.robot = robot;
    }

    @Override
    public MurphyState step() {
        Intake intake = robot.intake;
        Gamepad gamepad2 = robot.gamepad2;
        IMU imu = robot.imu;

        intake.driveSlide(gamepad2.left_stick_x, gamepad2.left_stick_y, imu.getRobotYawPitchRollAngles().getYaw(), gamepad2.right_stick_button);

        if (wristMoveTask != null) {
            if (!wristMoveTask.step(robot.telemetry)) {
                wristMoveTask = null;
            }
        } else if (gamepad2.right_stick_y < -0.15) {
            wristMoveTask = new Intake.WristTask(intake, Intake.WRIST_MID_POSITION);
        } else if (gamepad2.right_stick_y > 0.15) {
            wristMoveTask = new Intake.WristTask(intake, Intake.WRIST_DOWN_POSITION);
        }

        if (gamepad2.left_trigger > 0.1) {
            intake.setSpin(Intake.SPIN_IN);
        }

        if (gamepad2.x) {
            intake.setSpin(Intake.SPIN_STOP);
        }

        if (gamepad2.right_trigger > 0.1) {
            intake.setSpin(Intake.SPIN_STOP);
            return new TransferringState(robot, Outtake.HIGH_BASKET_POSITION);
        }

        if (gamepad2.right_bumper) {
            intake.setSpin(Intake.SPIN_STOP);
            return new TransferringState(robot, Outtake.LOW_BASKET_POSITION);
        }

        if (intake.hasSampleOfCorrectColor(robot.alliance) && !gamepad2.left_bumper) {
            intake.setSpin(Intake.SPIN_STOP);
        } else if (intake.hasSample()) {
            intake.setSpin(Intake.SPIN_OUT);
        }

        if (gamepad2.b) {
            return new SpecimenState(robot);
        }

        return this;
    }

    @Override
    public String getName() {
        return "Intaking";
    }
}
