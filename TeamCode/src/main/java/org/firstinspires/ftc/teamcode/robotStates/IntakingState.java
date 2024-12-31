package org.firstinspires.ftc.teamcode.robotStates;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.murphy.MurphyAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class IntakingState implements MurphyState {
    private final Robot robot;

    private MurphyAction wristMoveAction = null;

    public IntakingState(Robot robot) {
        this.robot = robot;
    }

    @Override
    public MurphyState step() {
        Intake intake = robot.intake;
        Gamepad gamepad2 = robot.gamepad2;
        IMU imu = robot.imu;

        intake.driveFieldCentric(gamepad2.left_stick_x, gamepad2.right_stick_y, imu.getRobotYawPitchRollAngles().getYaw());

        if (wristMoveAction != null) {
            if (wristMoveAction.step()) {
                wristMoveAction = null;
            }
        } else if (gamepad2.right_stick_y < -0.1) {
            wristMoveAction = new Intake.WristMidAction(intake);
        } else if (gamepad2.right_stick_y > 0.1) {
            wristMoveAction = new Intake.WristDownAction(intake);
        }

        if (gamepad2.left_trigger > 0.1) {
            intake.setSpin(Intake.SPIN_IN);
        }

        if (gamepad2.left_bumper) {
            intake.setSpin(Intake.SPIN_OUT);
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

        return this;
    }

    @Override
    public String getName() {
        return "Intaking";
    }
}
