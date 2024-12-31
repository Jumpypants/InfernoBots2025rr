package org.firstinspires.ftc.teamcode.robotStates;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.murphy.MurphyAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyParallelAction;
import org.firstinspires.ftc.teamcode.murphy.MurphySequentialAction;
import org.firstinspires.ftc.teamcode.murphy.MurphyState;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class OuttakingState implements MurphyState {
    private final Gamepad gamepad2;
    private final Intake intake;
    private final Outtake outtake;
    private final Robot robot;

    private final MurphyAction action;

    public OuttakingState(Robot robot, double targetPosition) {
        this.robot = robot;

        this.gamepad2 = robot.gamepad2;
        this.intake = robot.intake;
        this.outtake = robot.outtake;

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
        IMU imu = robot.imu;

        intake.driveFieldCentric(gamepad2.left_stick_x, gamepad2.right_stick_y, imu.getRobotYawPitchRollAngles().getYaw());

        if (!action.step()) {
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
