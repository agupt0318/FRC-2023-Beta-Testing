package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX; // import for victorspx
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm2 extends SubsystemBase {
    private final VictorSPX victorspx;
    private static Arm instance = null;

    public Arm2() {
        this.victorspx = new VictorSPX(0); // i don't actually know the motor port i'm just putting 0 to avoid compile error
    }

    // Singleton class, call getInstance to access instead of the constructor.
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    // method for the victorspx (the "arm", motor controlling the pendullum thing)
    public void setPercentOutputVictorSPX(double percent) {
        
        if(Math.abs(percent) < 0.08) {
            percent = 0;
        }
      
        if(percent == 0) {
            //victorspx.enableBrakeMode(true);    
        }
        System.out.println(percent);

        victorspx.set(ControlMode.PercentOutput, percent); 
        // works same as talon: input range on joystick y should map to the speed, where both the speed and xbox controller is in range -1,1
        // can also omit first argument for it to default to ControlMode.PercentOutput i believe
    }
}
