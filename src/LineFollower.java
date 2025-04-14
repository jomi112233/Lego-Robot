import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class LineFollower {
    private static final float KP = 500.0f;  // Reduced from 1000 for smoother control
    private static final float KI = 0.0f;
    private static final float KD = 50.0f;   // Reduced from 100 for smoother control
    private static final int BASE_SPEED = 150;
    private static final int MAX_SPEED = 300;
    private static final float OBSTACLE_DISTANCE = 0.5f;
    
    private static float integral = 0;
    private static float lastError = 0;
    private static float targetValue = 0.5f; // Default target value, will be calibrated

    public static void main(String[] args) {
        LCD.drawString("Press any button", 0, 0);
        LCD.drawString("to start", 0, 1);
        Button.waitForAnyPress();
        LCD.clear();

        // Create shared instances
        UltraThread ultraThread = new UltraThread();
        LineThread lineThread = new LineThread();

        // Start threads
        Thread ultraSensorThread = new Thread(ultraThread);
        Thread lineSensorThread = new Thread(lineThread);
        Thread motorThread = new Thread(new MotorThread(ultraThread, lineThread));

        ultraSensorThread.start();
        lineSensorThread.start();
        motorThread.start();
    }
}

class MotorThread implements Runnable {
    private UltraThread ultraThread;
    private LineThread lineThread;
    private float integral = 0;
    private float lastError = 0;

    public MotorThread(UltraThread ultraThread, LineThread lineThread) {
        this.ultraThread = ultraThread;
        this.lineThread = lineThread;
    }

    public void run() {
        while (!Button.ESCAPE.isDown()) {
            float distance = ultraThread.getDistance();
            float currentValue = lineThread.getLineValue();
            float targetValue = lineThread.getTargetValue();

            // Check for obstacles
            if (distance <= LineFollower.OBSTACLE_DISTANCE) {
                Motor.A.rotate(-360, true);
                Motor.B.rotate(360);
                Delay.msDelay(1000);
                continue;
            }

            // PID control for line following
            float error = targetValue - currentValue;
            integral = integral * 0.5f + error;
            float derivative = error - lastError;
            float correction = LineFollower.KP * error + LineFollower.KI * integral + LineFollower.KD * derivative;

            int speedA = (int)(LineFollower.BASE_SPEED + correction);
            int speedB = (int)(LineFollower.BASE_SPEED - correction);

            // Limit speeds
            speedA = Math.min(Math.max(speedA, -LineFollower.MAX_SPEED), LineFollower.MAX_SPEED);
            speedB = Math.min(Math.max(speedB, -LineFollower.MAX_SPEED), LineFollower.MAX_SPEED);

            // Set motor speeds and directions
            Motor.A.setSpeed(Math.abs(speedA));
            Motor.B.setSpeed(Math.abs(speedB));

            if (speedA > 0) {
                Motor.A.forward();
            } else {
                Motor.A.backward();
            }

            if (speedB > 0) {
                Motor.B.forward();
            } else {
                Motor.B.backward();
            }

            lastError = error;
            Delay.msDelay(10);
        }

        Motor.A.stop();
        Motor.B.stop();
    }
}

class UltraThread implements Runnable {
    private float distance = Float.MAX_VALUE;

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (!Button.ESCAPE.isDown()) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0];
            Delay.msDelay(40);
        }
        ultrasonicSensor.close();
    }
}

class LineThread implements Runnable {
    private float currentValue = 0.5f;
    private float targetValue = 0.5f;
    private boolean isCalibrated = false;

    public float getLineValue() {
        return currentValue;
    }

    public float getTargetValue() {
        return targetValue;
    }

    public void run() {
        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
        SampleProvider colorProvider = colorSensor.getRedMode();
        float[] colorSample = new float[colorProvider.sampleSize()];

        // Calibration
        LCD.drawString("Calibrating...", 0, 0);
        LCD.drawString("Place on line", 0, 1);
        Button.waitForAnyPress();
        colorProvider.fetchSample(colorSample, 0);
        float lineValue = colorSample[0];

        LCD.drawString("Place off line", 0, 0);
        Button.waitForAnyPress();
        colorProvider.fetchSample(colorSample, 0);
        float backgroundValue = colorSample[0];

        targetValue = (lineValue + backgroundValue) / 2;
        isCalibrated = true;
        LCD.clear();
        LCD.drawString("Following line", 0, 0);

        // Main line following loop
        while (!Button.ESCAPE.isDown()) {
            colorProvider.fetchSample(colorSample, 0);
            currentValue = colorSample[0];
            Delay.msDelay(10);
        }
        colorSensor.close();
    }
}
