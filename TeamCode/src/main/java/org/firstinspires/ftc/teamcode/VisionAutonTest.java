package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Robot;
import org.firstinspires.ftc.teamcode.vision.Sample;
import org.firstinspires.ftc.teamcode.vision.SampleFinder;

import java.util.ArrayList;

@Config
@Autonomous(name = "VisionAutonTest", group = "Autonomous")
public class VisionAutonTest extends LinearOpMode {
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Robot.Alliance alliance = Robot.Alliance.RED;
    private SampleFinder sampleFinder;
    private MecanumDrive drive;

    private Sample selectedSample;
    private double angleAtSelection;

    @Override
    public void runOpMode() {
        sampleFinder = new SampleFinder(hardwareMap, telemetry, new double[]{0, 0, 0});

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        while (!opModeIsActive()) {
            dashboardTelemetry.update();
        }

        Actions.runBlocking(find());

        while (opModeIsActive()) {
            dashboardTelemetry.update();
        }
    }

    private Action find () {
        return telemetryPacket -> {
            ArrayList<Sample> samples = sampleFinder.get(telemetry);

            if (samples.isEmpty()) return false;

            selectedSample = samples.get(0);

            angleAtSelection = drive.pose.heading.real;

            dashboardTelemetry.addData("current Angle", angleAtSelection);
            dashboardTelemetry.addData("selected Sample angle", selectedSample.getAngle());

            dashboardTelemetry.update();

            return false;
        };
    }

    private Action rotate () {
        return drive.actionBuilder(new Pose2d(0, 0, 0))
                .turnTo(Math.toRadians(selectedSample.getAngle() + angleAtSelection))
                .build();
    }
}