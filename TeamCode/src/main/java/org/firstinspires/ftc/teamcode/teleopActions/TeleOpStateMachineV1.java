package org.firstinspires.ftc.teamcode.teleopActions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

@Config
public class TeleOpStateMachineV1 {
    private final Intake INTAKE;
    private final Outtake OUTTAKE;
    private final Telemetry TELEMETRY;
    private final IMU IMU;
    private final Gamepad GAMEPAD1;
    private final Gamepad GAMEPAD2;
    private final SampleFinder SAMPLE_FINDER;

    private TeleOpAction currentAction;

    public TeleOpStateMachineV1 (
            Intake intake,
            Outtake outtake,
            Telemetry telemetry,
            IMU imu, Gamepad gamepad1,
            Gamepad gamepad2,
            SampleFinder sampleFinder
    ) {
        INTAKE = intake;
        OUTTAKE = outtake;
        TELEMETRY = telemetry;
        IMU = imu;
        GAMEPAD1 = gamepad1;
        GAMEPAD2 = gamepad2;
        SAMPLE_FINDER = sampleFinder;
    }

    public void step() {
        if (currentAction != null) currentAction.step();
    }
}