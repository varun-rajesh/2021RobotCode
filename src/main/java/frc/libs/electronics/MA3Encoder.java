package frc.libs.electronics;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.AnalogInput;

public class MA3Encoder {

    private int ticksPerRev;
    private int tickOverflowThreshold;
    
    private AnalogInput encoder;
    private int deviceID;
    private int moduleOffset;

    private int lastValue;
    private int overflowCounter;

    private Object encoderLock;

    public MA3Encoder(int deviceID, int moduleOffset) {
        this.encoder = new AnalogInput(deviceID);
        this.deviceID = deviceID;
        this.moduleOffset = moduleOffset;

        ticksPerRev = 4096;
        tickOverflowThreshold = 2048;

        this.lastValue = 0;
        this.overflowCounter = 0;

        encoderLock = new Object();

        ScheduledExecutorService encoderTracker = Executors.newScheduledThreadPool(1);
        encoderTracker.scheduleAtFixedRate(this::track, 0, 10, TimeUnit.MILLISECONDS);
    }

    private void track() {
        synchronized(encoderLock) {
            int currentValue = getOffsetValue();

            if(currentValue - lastValue < -tickOverflowThreshold) overflowCounter++;
            if(currentValue - lastValue > tickOverflowThreshold) overflowCounter--;

            lastValue = currentValue;
        }
    }

    public double getValue() {
        synchronized(encoderLock) {
            return ticksToRadians(ticksPerRev * overflowCounter + getOffsetValue());
        }
    }

    private int getOffsetValue() {
        return Math.floorMod((-encoder.getValue() + moduleOffset), ticksPerRev);
    }

    private double ticksToRadians(double ticks) {
        return ticks * (2 * Math.PI) / (ticksPerRev);
    }

    public void reset() {
        overflowCounter = 0;
    }

}