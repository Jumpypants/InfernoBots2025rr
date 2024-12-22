package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

// servo that controls a claw that has an in position and out position
public class SpecimenTake {

    private final CRServo SPECIMEN_CLAW;
    private static double OPEN_CLAW_POS;
    private static double CLOSE_CLAW_POS;



    public SpecimenTake(HardwareMap hardwareMap) {
        SPECIMEN_CLAW = new CRServo(hardwareMap, "claw");
    }

    // check if claw is closed
    public boolean closed(){
        return SPECIMEN_CLAW.getCurrentPosition() <= OPEN_CLAW_POS;
    }

    // increase servo position
    public void openClaw(){
        if (closed()){
            SPECIMEN_CLAW.set(1);
        }
    }

    // decrease servo position
    public void closeClaw(){
        if (!closed()){
            SPECIMEN_CLAW.set(-1);
        }
    }
}
