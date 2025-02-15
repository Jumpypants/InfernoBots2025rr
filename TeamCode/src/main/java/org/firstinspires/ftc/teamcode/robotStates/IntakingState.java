package org.firstinspires.ftc.teamcode.robotStates;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.murphy.MurphyTask;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;


@Config
public class IntakingState implements MurphyState {
    private final Robot robot;

    private MurphyTask kickTask = null;

    public IntakingState(Robot robot) {
        this.robot = robot;
    }

    @Override
    public MurphyState step() {
        Intake intake = robot.intake;
        Gamepad gamepad2 = robot.gamepad2;
        IMU imu = robot.imu;

        intake.driveSlide(gamepad2.left_stick_x, gamepad2.left_stick_y, imu.getRobotYawPitchRollAngles().getYaw(), gamepad2.left_bumper);

        robot.rotationOffset = gamepad2.left_stick_x / 6;


        double range = Intake.WRIST_MID_POSITION - Intake.WRIST_DOWN_POSITION;

        double offset = Intake.WRIST_MID_POSITION + (intake.getIsFlippedLow() ? 0 : Intake.TOP_DOWN_WRIST_OFFSET);

        intake.setWrist(-gamepad2.right_stick_y * range + offset);


        if (kickTask != null) {
            if (!kickTask.step(robot.telemetry)) {
                kickTask = null;
            }
        } else if (gamepad2.y) {
            kickTask = new Kicker.KickTask(robot.kicker);
        }

        if (gamepad2.right_stick_x > 0.5) {
            intake.setFlip(Intake.FLIP_HIGH_POSITION);
        } else if (gamepad2.right_stick_x < -0.5) {
            intake.setFlip(Intake.FLIP_LOW_POSITION);
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

        if (intake.hasSampleOfCorrectColor(robot.alliance) && !gamepad2.right_stick_button) {
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
