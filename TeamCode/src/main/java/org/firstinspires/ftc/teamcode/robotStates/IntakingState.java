package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.murphy.MurphyAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class IntakingState implements MurphyState {
    private final Gamepad gamepad2;
    private final Intake intake;
    private final Outtake outtake;

    private MurphyAction wristMoveAction = null;

    public IntakingState(Gamepad gamepad2, Intake intake, Outtake outtake) {
        this.gamepad2 = gamepad2;
        this.intake = intake;
        this.outtake = outtake;
    }

    @Override
    public MurphyState step() {
        intake.setSlideSetPoint(intake.getSlidePosition() - gamepad2.left_stick_y * 0.75);

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
            return new TransferringState(gamepad2, intake, outtake, Outtake.HIGH_BASKET_POSITION);
        }

        if (gamepad2.right_bumper) {
            intake.setSpin(Intake.SPIN_STOP);
            return new TransferringState(gamepad2, intake, outtake, Outtake.LOW_BASKET_POSITION);
        }

        return this;
    }

    @Override
    public String getName() {
        return "Intaking";
    }
}
