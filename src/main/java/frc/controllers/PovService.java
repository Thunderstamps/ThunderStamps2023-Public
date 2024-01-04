package frc.controllers;

import edu.wpi.first.wpilibj.XboxController;

public class PovService {

    public static final int POV_NONE = -1;
    private static final double lineOrientations[] = {
        Math.toRadians(0),   // 000 (dpad POV)
        Math.toRadians(-45),  // 045
        Math.toRadians(-90),  // 090
        Math.toRadians(-135), // 135
        Math.toRadians(180),// 180
        Math.toRadians(135),// 225
        Math.toRadians(90), // 270
        Math.toRadians(45), // 315
    };

    private final Thread backgroundThread;

    private final Object syncLock = new Object();
    private int pov = POV_NONE;
    private int lastPov = 0; // initialize to forward so that if the driver presses a bumper button it still works

    public PovService(XboxController xboxController) {
        backgroundThread = new Thread(() -> {
            while (!Thread.interrupted()) {
                setPOV(xboxController.getPOV());
                try {
                    Thread.sleep(20);
                } catch (InterruptedException e) {
                }
            }
        });
        backgroundThread.start();
    }

    private void setPOV(int newPOV) {
        if(newPOV > POV_NONE) {
            synchronized(syncLock){
                this.pov = newPOV;
                this.lastPov = newPOV;
            }
        }
    }

    public void setPOV90FromDegrees(double degrees) {
        if(degrees >= -45 && degrees <= 45) {
            this.setPOV(0);
        }
        else if(degrees >= -135 && degrees <= -45) {
            this.setPOV(90);
        }
        else if(degrees >= 45 && degrees <= 135) {
            this.setPOV(270);
        }
        else {
            this.setPOV(180);
        }
    }

    public void setPOV45FromDegrees(double degrees) {
        if(degrees >= -22.5 && degrees <= 22.5) {
            this.setPOV(0);
        }
        else if(degrees >= -45-22.5 && degrees <= -45+22.5) {
            this.setPOV(45);
        }
        else if(degrees >= -90-22.5 && degrees <= -90+22.5) {
            this.setPOV(90);
        }
        else if(degrees >= -135-22.5 && degrees <= -135+22.5) {
            this.setPOV(135);
        }
        else if(degrees >= 45-22.5 && degrees <= 45+22.5) {
            this.setPOV(315);
        }
        else if(degrees >= 90-22.5 && degrees <= 90+22.5) {
            this.setPOV(270);
        }
        else if(degrees >= 135-22.5 && degrees <= 135+22.5) {
            this.setPOV(225);
        }
        else {
            this.setPOV(180);
        }
    }

    public int getPOV() {
        int result;
        synchronized(syncLock){
            result = this.pov;
            this.pov = POV_NONE;
        }
        return result;
    }

    public int getNextPOV() {
        int result = POV_NONE;
        synchronized(syncLock){
            if(this.lastPov > POV_NONE) {
                this.lastPov = this.lastPov + 45;
                if(this.lastPov >= 360) {
                    this.lastPov = 0;
                }
                result = this.lastPov;
            }
        }
        return result;
    }

    public int getPrevPOV() {
        int result = POV_NONE;
        synchronized(syncLock){
            if(this.lastPov > POV_NONE) {
                this.lastPov = this.lastPov - 45;
                if(this.lastPov < 0) {
                    this.lastPov += 360;
                }
                result = this.lastPov;
            }
        }
        return result;
    }

    public void resetLastPOV() {
        synchronized(syncLock){
            this.lastPov = POV_NONE;
        }
    }

    public void zeroLastPOV() {
        synchronized(syncLock){
            this.lastPov = 0;
        }
    }

    public double getSnapAngleFromPovRadians(int pov) {
        // pov is a number in range 0, 45, 90, 135, 180, 225, 270, 315
        var index = getSnapIndex(pov);
        return lineOrientations[index];
    }

    // returns a number from 0 to 7 based on the POV (dpad) direction
    private int getSnapIndex(int pov) {
        // pov is a number in range 0, 45, 90, 135, 180, 225, 270, 315
        switch(pov) {
            case 45: return 1; 
            case 90: return 2; 
            case 135: return 3; 
            case 180: return 4; 
            case 225: return 5; 
            case 270: return 6; 
            case 315: return 7; 
        }
        return 0;
    }
}