package org.firstinspires.ftc.teamcode.murphy;

public class MurphyParallelAction extends MurphyAction {
    private final MurphyAction[] actions;

    private final boolean stopOnFirstCompletion;

    public MurphyParallelAction(boolean stopOnFirstCompletion, MurphyAction... actions) {
        super();
        this.actions = actions;
        this.stopOnFirstCompletion = stopOnFirstCompletion;
    }

    @Override
    public boolean step() {
        boolean runAgain = false;

        for (MurphyAction action : actions) {
            if (action.step()) {
                runAgain = true;
                if (stopOnFirstCompletion) {
                    break;
                }
            }
        }

        return runAgain;
    }
}
