package frc.robot.utilities;

import java.time.*;

public class OnDelayTimer {

    private final long delay_ms;
    private boolean lastValue;
    private Instant risingEdgeTime = Instant.now();
    private boolean output = false;

    public OnDelayTimer(long delay_ms) {
        this.delay_ms = delay_ms;
    }

    public boolean execute(boolean input) {

        if(!lastValue && input) {
            risingEdgeTime = Instant.now();
        }
        lastValue = input;

        if(input) {
            var duration = Duration.between(risingEdgeTime, Instant.now());
            output = duration.toMillis() >= this.delay_ms;
        }
        else {
            output = false;
        }
        return output;
    }

    public boolean getOutput() {
        return this.output;
    }
}