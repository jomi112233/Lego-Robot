import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class evade {
    public static void main(String[] args) {
        UltraThread ultraThread = new UltraThread(); // Shared instance

        Thread motorThread = new Thread(new MotorThread(ultraThread)); // Pass it to MotorThread
        Thread ultraSensorThread = new Thread(ultraThread);

        ultraSensorThread.start();
        motorThread.start();
    }
}

class MotorThread implements Runnable {
    private UltraThread ultraThread;
    private int turnCounter = 0; // Tracks how many right turns were made
    private int forwardCounter = 0; // Tracks how many forward movements were made

    public MotorThread(UltraThread ultraThread) {
        this.ultraThread = ultraThread;
    }

    public void run() {
        while (true) {
            float distance = ultraThread.getDistance(); // Get the current distance from the sensor

            // Drive forward by default
            Motor.A.forward();
            Motor.B.forward();

            if (distance <= 0.5f) { // Obstacle detected
                // Turn right
                Motor.A.rotate(-180, true); // Rotate right
                Motor.B.rotate(180);
                turnCounter++; // Increment turn counter

                // Move forward a small distance
                Motor.A.rotate(500, true);
                Motor.B.rotate(500);
                forwardCounter++; // Increment forward counter

                try {
                    Thread.sleep(100); // Pause briefly
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                // Check if the path is clear again
                if (ultraThread.getDistance() >= 0.5f) {
                    // Turn back forward to check if the obstacle is still there
                    Motor.A.rotate(180, true);
                    Motor.B.rotate(-180);

                    if (ultraThread.getDistance() >= 0.5f) {
                        // Path is clear, return to the original path
                        returnToOriginalPath();
                    }
                }
            }
        }
    }

    private void returnToOriginalPath() {
        // Reverse the forward movements
        for (int i = 0; i < forwardCounter; i++) {
            Motor.A.rotate(-500, true);
            Motor.B.rotate(-500);
        }

        // Reverse the right turns
        for (int i = 0; i < turnCounter; i++) {
            Motor.A.rotate(180, true);
            Motor.B.rotate(-180);
        }

        // Reset counters after returning to the original path
        turnCounter = 0;
        forwardCounter = 0;
    }
}

class UltraThread implements Runnable  {
    private float distance = Float.MAX_VALUE; 

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (true) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0]; // Update shared variable

            //LCD.clear();
            //LCD.drawString("Distance: " + distance, 0, 0);

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }     
    }
}
