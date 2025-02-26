package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Elevator;

public class SetHeight extends InstantCommand {
    private Elevator m_elevator;
    private double m_height;

    public SetHeight (double height, Elevator elevator) {
        m_elevator = elevator;
        m_height = height;
        addRequirements(m_elevator);
    }

    public void initialize() {
        m_elevator.EnablePID();
        m_elevator.setheight(m_height);
        System.out.println("Elev PID seen, set point = " + m_height);
    }
}
