package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

public class NoneState implements MurphyState {
    private final Gamepad gamepad2;
    private final Intake intake;
    private final Outtake outtake;

    private Intake.WristDownAction wristDownAction = null;

    public NoneState(Gamepad gamepad2, Intake intake, Outtake outtake) {
        this.gamepad2 = gamepad2;
        this.intake = intake;
        this.outtake = outtake;
    }

    @Override
    public MurphyState step() {
        intake.setSlideSetPoint(intake.getSlidePosition() - gamepad2.left_stick_y * 0.75);

        if (wristDownAction != null) {
            if (wristDownAction.step()) {
                return new IntakingState(gamepad2, intake, outtake);
            }
        } else if (gamepad2.right_stick_y > 0.1) {
            wristDownAction = new Intake.WristDownAction(intake);
        }
        return this;
    }

    @Override
    public String getName() {
        return "None";
    }
}
