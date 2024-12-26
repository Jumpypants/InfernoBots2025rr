package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.murphy.MurphyAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyParallelAction;
import org.firstinspires.ftc.teamcode.murphy.MurphySequentialAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class OuttakingState implements MurphyState {
    private final Gamepad gamepad2;
    private final Intake intake;
    private final Outtake outtake;

    private final MurphyAction action;

    public OuttakingState(Gamepad gamepad2, Intake intake, Outtake outtake, double targetPosition) {
        this.gamepad2 = gamepad2;
        this.intake = intake;
        this.outtake = outtake;

        action = new MurphySequentialAction(
                new Intake.ClearWristAction(intake),
                new MurphyParallelAction( false,
                        new Outtake.MoveSlideAction(outtake, targetPosition),
                        new Outtake.SpinToMidAction(outtake)
                ),
                new Outtake.DumpAction(outtake, gamepad2)
        );
    }

    @Override
    public MurphyState step() {
        intake.setSlideSetPoint(intake.getSlidePosition() - gamepad2.left_stick_y * 0.75);

        if (!action.step()) {
            outtake.setSlideSetPoint(Outtake.DOWN_POSITION);
            outtake.setSpin(Outtake.SPIN_IN_POSITION);
            return new NoneState(gamepad2, intake, outtake);
        }
        return this;
    }

    @Override
    public String getName() {
        return "Outtaking";
    }
}
