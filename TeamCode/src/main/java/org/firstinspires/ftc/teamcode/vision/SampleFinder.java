package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Comparator;

public class SampleFinder {
    public static final double MIN_STONE_AREA = 500;

    private final double[] cameraPosition;

    OpenCvCamera camera;
    SampleDetectionPipelinePNP pipeline;

    public SampleFinder (HardwareMap hardwareMap, Telemetry telemetry, double[] cameraPosition) {
        this.cameraPosition = cameraPosition;

        // Initialize OpenCV camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "33DDE480", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Initialize pipeline
        pipeline = new SampleDetectionPipelinePNP();
        camera.setPipeline(pipeline);

        // Start the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Error", "Error code: %d", errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
    }

    public ArrayList<Sample> get(Telemetry telemetry) {
        ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> stones = pipeline.getDetectedStones();

        // Filter out stones that are too small
        stones.removeIf(stone -> stone.rect.size.area() < MIN_STONE_AREA);

        // Convert the stones to samples
        ArrayList<Sample> samples = getSampleList(stones);

        // Sort the stones by y distance in ascending order
        samples.sort(Comparator.comparingDouble(Sample::getY));

        // Print the samples
        for (int i = 0; i < samples.size(); i++) {
            telemetry.addData(i + " ", samples.get(i).toString());
        }

        return samples;
    }

    private ArrayList<Sample> getSampleList (ArrayList<SampleDetectionPipelinePNP.AnalyzedStone> stones) {
        ArrayList<Sample> samples = new ArrayList<>();

        for (SampleDetectionPipelinePNP.AnalyzedStone stone : stones) {
            samples.add(new Sample(
                    getTranslationVector(stone)[0] - cameraPosition[0],
                    getTranslationVector(stone)[1] - cameraPosition[1],
                    getTranslationVector(stone)[2] - cameraPosition[2],
                    stone.color, stone.rect
            ));
        }

        return samples;
    }

    private double[] getTranslationVector(SampleDetectionPipelinePNP.AnalyzedStone stone) {
        return new double[]{stone.tvec.get(0, 0)[0], stone.tvec.get(1, 0)[0], stone.tvec.get(2, 0)[0]};
    }
}
