package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS; 

public class GyroSubystem extends SubsystemBase {
    private final static AHRS gyro = new AHRS(SPI.Port.kMXP);
    private static double offsetAngle = 0;
    private static double gyroDirection = 1;
    public GyroSubystem(){
        offsetAngle = 0;
        calibrateGyro();
    }   
    // Callibrates gyro
    public void calibrateGyro() {
      gyro.calibrate();
    }

    // Reset the gyro Yaw and reconnect -> Used for restart
    public void zeroGyro() {
      System.out.println("NavX Connected: " + gyro.isConnected());
      gyro.reset();
    }

    // 	Returns the current yaw, pitch, and roll value (in degrees, from -180 to 180) reported by the sensor. 
    public double getYaw() {
      return gyro.getYaw();
    }
    public double getPitch() {
      return gyro.getPitch();
    }
    public double getRoll() {
      return gyro.getRoll();
    }
    // Returns accumulate yaw angle
    public double getAngle() {
      return gyro.getAngle();
    }

    public void setAngleAdjustment(double adjustment){
      // Sets angle adjustment
      gyro.setAngleAdjustment(adjustment);
  }

    public void toggleDirection() {
        // Changes gyro direction
        gyroDirection *= -1;
    }

    public void resetDisplacement(){
      // Sets the integration variables of the displacment calculations to 0(Sets point for displacment to begin calculation)
        gyro.resetDisplacement();
    }
    
    public double getDispY(){ // Returns the displacement (in meters) of the Y axis since resetDisplacement() was last invoked 
      return gyro.getDisplacementY();
    }

    public double getDispX(){ //Returns the displacement (in meters) of the X axis since resetDisplacement() was last invoked
      return gyro.getDisplacementX();
    }

    public double getVeloX(){  //	Returns the velocity (in meters/sec) of the X axis
      return gyro.getVelocityX();
    }
    public double getVeloY(){ // 	Returns the velocity (in meters/sec) of the Y axis
      return gyro.getVelocityY();
    }

    public static double getGyroAccelX(){ // Returns the current linear acceleration in the X-axis (in G).
        return gyro.getWorldLinearAccelX();
      }
    
    public static double getGyroAccelY(){ // Returns the current linear acceleration in the Y-axis (in G).
        return gyro.getWorldLinearAccelY();
    }
    
    public static void setAngleOffset(double offsetDeg) {
        offsetAngle = offsetDeg;
    }
}