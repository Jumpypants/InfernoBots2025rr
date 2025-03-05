package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.murphy.MurphyTask;
import org.firstinspires.ftc.teamcode.murphy.MurphyParallelTask;
import org.firstinspires.ftc.teamcode.murphy.MurphySequentialTask;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakingState implements MurphyState {
    private final Gamepad gamepad2;
    private final Intake intake;
    private final Outtake outtake;
    private final Robot robot;

    private final MurphyTask mainTask;

    public OuttakingState(Robot robot, double targetPosition) {
        this.robot = robot;

        this.gamepad2 = robot.gamepad2;
        this.intake = robot.intake;
        this.outtake = robot.outtake;

        mainTask = new MurphySequentialTask(
                new Intake.ClearWristTask(intake),
                    new Outtake.MoveSlideTask(outtake, targetPosition),
                    new Outtake.SpinToMidTask(outtake),
                new Outtake.DumpTask(outtake, gamepad2),
                new Outtake.SpinToInTask(outtake)
        );
    }

    @Override
    public MurphyState step() {
        IMU imu = robot.imu;

        intake.driveSlide(gamepad2.left_stick_x, gamepad2.left_stick_y, imu.getRobotYawPitchRollAngles().getYaw(), !gamepad2.left_bumper);

        if (!mainTask.step(robot.telemetry) || gamepad2.left_trigger > 0.2) {
            outtake.setSlideSetPoint(Outtake.DOWN_POSITION);
            outtake.setSpin(Outtake.SPIN_IN_POSITION);
            return new IntakingState(robot);
        }

        return this;
    }

    @Override
    public String getName() {
        return "Outtaking";
    }
}
