package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.murphy.MurphyAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyParallelAction;
import org.firstinspires.ftc.teamcode.murphy.MurphySequentialAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class TransferringState implements MurphyState {
    private final Gamepad gamepad2;
    private final Intake intake;
    private final Outtake outtake;

    private final double targetPosition;

    private final MurphyAction action;

    public TransferringState(Gamepad gamepad2, Intake intake, Outtake outtake, double targetPosition) {
        this.gamepad2 = gamepad2;
        this.intake = intake;
        this.outtake = outtake;
        this.targetPosition = targetPosition;

        action = new MurphySequentialAction(
                new MurphyParallelAction( false,
                        new Intake.WristUpAction(intake),
                        new Intake.MoveSlideAction(intake, Intake.SLIDE_IN_POSITION)
                ),
                new Intake.TransferSpinAction(intake)
        );
    }

    @Override
    public MurphyState step() {
        return !action.step() ? new OuttakingState(gamepad2, intake, outtake, targetPosition) : this;
    }

    @Override
    public String getName() {
        return "Transferring";
    }
}
