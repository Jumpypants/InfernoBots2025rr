package org.firstinspires.ftc.teamcode.robotStates;

import com.fremontrobotics.murphy.SequentialTask;
import com.fremontrobotics.murphy.State;
import com.fremontrobotics.murphy.Task;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class SpecimenState implements State {
    private final Robot robot;
    private Task mainTask = null;

    public SpecimenState (Robot robot) {
        this.robot = robot;
        robot.intake.setWrist(Intake.WRIST_MID_POSITION);
    }

    @Override
    public State step() {
        if (mainTask != null) {
            if (!mainTask.step(robot.telemetry)) mainTask = null;
        } else {
            if (robot.gamepad2.right_bumper) mainTask = setUp();
            if (robot.gamepad2.right_trigger > 0.2) mainTask = grab();
            if (robot.gamepad2.left_trigger > 0.2) mainTask = place();
        }

        if (robot.gamepad2.y) {
            robot.outtake.setSlideSetPoint(Outtake.DOWN_POSITION);
            return new IntakingState(robot);
        }

        return this;
    }

    @Override
    public String getName() {
        return "Specimen";
    }

    private Task setUp () {
        return new SequentialTask(
                new Outtake.ClawOpenTask(robot.outtake),
                new Outtake.MoveSlideTask(robot.outtake, Outtake.DOWN_POSITION)
        );
    }

    private Task grab () {
        return new SequentialTask(
                new Outtake.ClawCloseTask(robot.outtake),
                new Outtake.MoveSlideTask(robot.outtake, Outtake.SPECIMEN_UP_POSITION)
        );
    }

    private Task place () {
        return new Outtake.MoveSlideTask(robot.outtake, Outtake.SPECIMEN_DOWN_POSITION);
    }
}
