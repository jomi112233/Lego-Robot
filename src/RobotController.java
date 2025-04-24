import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class RobotController {
    private static final float KP = 800.0f;
    private static final float KD = 120.0f;
    private static final int BASE_SPEED = 250;
    private static final float OBSTACLE_DISTANCE = 0.5f;

    private static float lastError = 0;

    public static void main(String[] args) {
        LCD.drawString("Press to start", 0, 0);
        Button.waitForAnyPress();
        LCD.clear();

        UltraThread ultraThread = new UltraThread();
        Thread ultraSensorThread = new Thread(ultraThread);
        ultraSensorThread.setDaemon(true); // Ensures thread exits with main program
        ultraSensorThread.start();

        EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
        SampleProvider sp = colorSensor.getRedMode();
        float[] sample = new float[sp.sampleSize()];
        float threshold = 0.3f; // Adjust this for your lighting/surface

        while (true) {
            float distance = ultraThread.getDistance();

            // Obstacle detected
            if (distance <= OBSTACLE_DISTANCE) {
                avoidObstacle();
                continue;
            }

            // Line following
            sp.fetchSample(sample, 0);
            float lightValue = sample[0];
            float error = threshold - lightValue;
            float derivative = error - lastError;
            lastError = error;

            float turn = KP * error + KD * derivative;

            int leftSpeed = clampSpeed(BASE_SPEED + (int) turn);
            int rightSpeed = clampSpeed(BASE_SPEED - (int) turn);

            Motor.A.setSpeed(leftSpeed);
            Motor.B.setSpeed(rightSpeed);
            Motor.A.forward();
            Motor.B.forward();
        }
    }

    private static int clampSpeed(int speed) {
        return Math.max(100, Math.min(speed, 600));
    }

    private static void avoidObstacle() {
        // Stop and turn
        Motor.A.stop(true);
        Motor.B.stop();
        Delay.msDelay(200);

        Motor.A.rotate(-180, true);
        Motor.B.rotate(180);
        Delay.msDelay(300);

        Motor.A.rotate(360, true);
        Motor.B.rotate(360);
        Delay.msDelay(500);
    }
}

class UltraThread implements Runnable {
    private volatile float distance = Float.MAX_VALUE;

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (true) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0];
            Delay.msDelay(30); // Balanced speed
        }
    }
}
﻿
