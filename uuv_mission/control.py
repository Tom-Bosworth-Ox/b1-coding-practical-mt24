from dataclasses import dataclass

@dataclass
class Controller:
    kp: float  # Proportional gain
    kd: float  # Derivative gain
    prev_error: float = 0.0  # Store previous error for derivative calculation
    dt: float = 1.0  # Time step, matching Submarine's default dt

    def compute_action(self, reference: float, current_depth: float) -> float:
        """Compute control action using PD control law.
        
        Args:
            reference (float): Desired depth
            current_depth (float): Current submarine depth
            
        Returns:
            float: Control action
        """
        # Calculate current error
        error = reference - current_depth
        
        # Calculate error derivative (change in error)
        error_derivative = (error - self.prev_error) / self.dt
        
        # Store current error for next iteration
        self.prev_error = error
        
        # PD control law
        action = self.kp * error + self.kd * error_derivative
        
        return action
    
    def reset(self):
        """Reset controller state."""
        self.prev_error = 0.0