package org.firstinspires.ftc.teamcode.murphy;

public class MurphySequentialAction extends MurphyAction {
    private final MurphyAction[] actions;

    private int currentActionIndex = 0;

    public MurphySequentialAction(MurphyAction... actions) {
        super();
        this.actions = actions;
    }

    @Override
    public boolean step() {
        if (currentActionIndex >= actions.length) {
            return false;
        }

        if (!actions[currentActionIndex].step()) {
            currentActionIndex++;
        }

        return true;
    }
}
