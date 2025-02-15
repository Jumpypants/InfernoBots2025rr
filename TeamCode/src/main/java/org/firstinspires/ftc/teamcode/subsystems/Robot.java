package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    public static enum Alliance {
        RED,
        BLUE
    }

    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    public final Intake intake;
    public final Outtake outtake;
    public final Kicker kicker;
    public final MecanumDrive driveBase;
    public final IMU imu;

    public final Telemetry telemetry;

    public Alliance alliance;

    public double rotationOffset = 0;

    public Robot(Gamepad gamepad1, Gamepad gamepad2, Intake intake, Outtake outtake, MecanumDrive driveBase, IMU imu, Telemetry telemetry, Alliance alliance, Kicker kicker) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
        this.intake = intake;
        this.outtake = outtake;
        this.driveBase = driveBase;
        this.imu = imu;
        this.telemetry = telemetry;
        this.alliance = alliance;
        this.kicker = kicker;
    }
}
