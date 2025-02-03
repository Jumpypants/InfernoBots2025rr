package org.firstinspires.ftc.teamcode.robotStates;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.murphy.MurphyParallelTask;
import org.firstinspires.ftc.teamcode.murphy.MurphySequentialTask;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.murphy.MurphyTask;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class SpecimenState implements MurphyState {
    private final Robot robot;
    private MurphyTask mainTask = null;

    public SpecimenState (Robot robot) {
        this.robot = robot;
        robot.intake.setWrist(Intake.WRIST_MID_POSITION);
    }

    @Override
    public MurphyState step() {
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

    private MurphyTask setUp () {
        return new MurphySequentialTask(
                new Outtake.ClawOpenTask(robot.outtake),
                new Outtake.MoveSlideTask(robot.outtake, Outtake.DOWN_POSITION)
        );
    }

    private MurphyTask grab () {
        return new MurphySequentialTask(
                new Outtake.ClawCloseTask(robot.outtake),
                new Outtake.MoveSlideTask(robot.outtake, Outtake.SPECIMEN_UP_POSITION)
        );
    }

    private MurphyTask place () {
        return new Outtake.MoveSlideTask(robot.outtake, Outtake.SPECIMEN_DOWN_POSITION);
    }
}
