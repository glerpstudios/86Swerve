package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorInput {
    private Joystick js = new Joystick(1);

    public Trigger js1 = new JoystickButton(js, 1);
    public Trigger js2 = new JoystickButton(js, 2);

    public OperatorInput() {
        
    }
}
