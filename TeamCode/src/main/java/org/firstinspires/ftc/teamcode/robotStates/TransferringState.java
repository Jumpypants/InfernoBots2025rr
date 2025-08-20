package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.fremontrobotics.murphy.State;
import com.fremontrobotics.murphy.Task;
import com.fremontrobotics.murphy.SequentialTask;
import com.fremontrobotics.murphy.ParallelTask;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class TransferringState implements State {
    private final Robot robot;

    private final double targetPosition;

    private final Task mainTask;

    public TransferringState(Robot robot, double targetPosition) {
        this.robot = robot;

        Intake intake = robot.intake;
        this.targetPosition = targetPosition;

        mainTask = new SequentialTask(
                new ParallelTask( false,
                        new Intake.WristTask(intake, Intake.WRIST_TRANSFER_POSITION),
                        new Intake.MoveSlideTask(intake, Intake.SLIDE_IN_POSITION),
                        new Intake.FlipTask(intake, Intake.FLIP_LOW_POSITION)
                ),
                new Intake.TransferSpinTask(intake)
        );
    }

    @Override
    public State step() {
        Gamepad gamepad2 = robot.gamepad2;
        if (gamepad2.left_trigger > 0.2) {
            robot.intake.setWrist(Intake.WRIST_MID_POSITION);
            return new IntakingState(robot);
        }
        return !mainTask.step(robot.telemetry) ? new OuttakingState(robot, targetPosition) : this;
    }

    @Override
    public String getName() {
        return "Transferring";
    }
}
