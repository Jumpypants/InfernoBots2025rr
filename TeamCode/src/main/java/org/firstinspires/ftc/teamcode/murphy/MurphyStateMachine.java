package org.firstinspires.ftc.teamcode.murphy;

public class MurphyStateMachine {
    private MurphyState currentState;

    public MurphyStateMachine(MurphyState initialState) {
        currentState = initialState;
    }

    public void step() {
        currentState = currentState.step();
    }
}
