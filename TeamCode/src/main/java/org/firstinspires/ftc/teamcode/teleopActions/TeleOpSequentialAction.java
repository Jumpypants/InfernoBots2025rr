package org.firstinspires.ftc.teamcode.teleopActions;

public class TeleOpSequentialAction extends TeleOpAction {
    private final TeleOpAction[] actions;

    private int currentActionIndex = 0;

    public TeleOpSequentialAction(TeleOpAction... actions) {
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
