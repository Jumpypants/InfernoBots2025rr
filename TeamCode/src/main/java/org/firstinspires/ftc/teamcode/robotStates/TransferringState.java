package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.murphy.MurphyAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyParallelAction;
import org.firstinspires.ftc.teamcode.murphy.MurphySequentialAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class TransferringState implements MurphyState {
    private final Robot robot;

    private final double targetPosition;

    private final MurphyAction action;

    public TransferringState(Robot robot, double targetPosition) {
        this.robot = robot;

        Intake intake = robot.intake;
        this.targetPosition = targetPosition;

        action = new MurphySequentialAction(
                new MurphyParallelAction( false,
                        new Intake.WristAction(intake, Intake.WRIST_UP_POSITION),
                        new Intake.MoveSlideAction(intake, Intake.SLIDE_IN_POSITION)
                ),
                new Intake.TransferSpinAction(intake)
        );
    }

    @Override
    public MurphyState step() {
        Gamepad gamepad2 = robot.gamepad2;
        if (Math.abs(gamepad2.left_stick_y) + Math.abs(gamepad2.left_stick_x) > 0.1) return new IntakingState(robot);
        return !action.step(robot.telemetry) ? new OuttakingState(robot, targetPosition) : this;
    }

    @Override
    public String getName() {
        return "Transferring";
    }
}
