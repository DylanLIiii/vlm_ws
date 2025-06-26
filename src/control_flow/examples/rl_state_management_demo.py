#!/usr/bin/env python3
"""
RL State Management Demonstration

This script demonstrates the improved persistent RL state management in the ActionExecutor.
It shows how the system now avoids unnecessary RL mode transitions when already in RL mode.

Usage:
    python3 rl_state_management_demo.py

The script will simulate various command sequences and show how RL state is managed.
"""

import time
from enum import Enum, auto


class TaskState(Enum):
    """State machine for task execution"""
    IDLE = auto()
    RL_MODE = auto()
    EXECUTING_TASK = auto()


class RLState(Enum):
    """Robot's current operational mode state"""
    RL_MODE = auto()
    STANDING = auto()
    SITTING = auto()
    SHAKING_HANDS = auto()


class RLStateDemo:
    """Simplified demonstration of RL State Management logic"""
    
    def __init__(self):
        # RL State Management settings
        self.current_rl_state = RLState.STANDING
        self.current_task_state = TaskState.IDLE
        
        # Commands that require RL mode for execution
        self.rl_mode_required_commands = {
            'follow_start', 'Come Here', 'Come to my front',
            'Come to my behind/back', 'Come to my left', 'Come to my right'
        }
        
        # Commands that interrupt RL mode and transition to non-RL states
        self.rl_interrupting_commands = {
            'stand_up': RLState.STANDING,
            'stand_down': RLState.SITTING,
            'shake_hand': RLState.SHAKING_HANDS
        }
        
        # RL-based commands that stay in RL mode after completion
        self.rl_commands_stay_in_rl = {
            'follow_start', 'Come Here', 'Come to my front', 'Come to my behind/back',
            'Come to my left', 'Come to my right'
        }
        
    def log(self, message):
        """Simple logging function"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] {message}")
        
    def should_skip_rl_entry(self, command):
        """Determine if RL mode entry should be skipped for a command."""
        return (command in self.rl_mode_required_commands and 
                self.current_rl_state == RLState.RL_MODE)
        
    def log_rl_state_transition(self, command, action_taken):
        """Log RL state transitions for debugging and monitoring."""
        self.log(f"RL State Management - Command: '{command}', "
                f"Current RL State: {self.current_rl_state.name}, "
                f"Task State: {self.current_task_state.name}, "
                f"Action: {action_taken}")
        
    def simulate_command(self, command):
        """Simulate processing a command with improved RL state management"""
        self.log(f"Processing command: '{command}'")
        
        # Handle RL interrupting commands
        if command in self.rl_interrupting_commands:
            target_rl_state = self.rl_interrupting_commands[command]
            self.log_rl_state_transition(command, f"Interrupting RL mode - transitioning to {target_rl_state.name}")
            self.current_rl_state = target_rl_state
            self.current_task_state = TaskState.EXECUTING_TASK
            
        # Handle commands that require RL mode
        elif command in self.rl_mode_required_commands:
            if self.should_skip_rl_entry(command):
                self.log_rl_state_transition(command, "Skipping RL entry - already in RL mode")
                self.current_task_state = TaskState.EXECUTING_TASK
            else:
                self.log_rl_state_transition(command, f"Entering RL mode from {self.current_rl_state.name}")
                self.current_rl_state = RLState.RL_MODE
                self.current_task_state = TaskState.RL_MODE
                
        # Handle other commands
        else:
            self.log_rl_state_transition(command, "Executing directly in current state")
            self.current_task_state = TaskState.EXECUTING_TASK
            
        # Simulate task execution
        time.sleep(0.1)
        self.simulate_task_completion(command)
        
    def simulate_task_completion(self, command):
        """Simulate task completion with improved RL state persistence"""
        if command in self.rl_commands_stay_in_rl:
            self.log(f"Task '{command}' completed - staying in RL mode, transitioning to idle")
            # Maintain RL state but transition to idle (don't re-enter RL mode)
            self.current_task_state = TaskState.IDLE
        else:
            self.log(f"Task '{command}' completed - transitioning to idle")
            self.current_task_state = TaskState.IDLE
            
    def get_state_summary(self):
        """Get current state summary"""
        return {
            'task_state': self.current_task_state.name,
            'rl_state': self.current_rl_state.name,
            'is_in_rl_mode': self.current_rl_state == RLState.RL_MODE,
            'persistent_rl_active': (self.current_rl_state == RLState.RL_MODE and 
                                   self.current_task_state == TaskState.IDLE)
        }


def main():
    """Main demonstration function"""
    demo = RLStateDemo()
    
    print("=" * 70)
    print("RL State Management Improvement Demonstration")
    print("=" * 70)
    
    # Demo 1: First RL command - should enter RL mode
    print("\n1. First RL Command - Should Enter RL Mode")
    print("-" * 50)
    
    demo.simulate_command("Come Here")
    state = demo.get_state_summary()
    print(f"State after first RL command: {state}")
    
    # Demo 2: Second RL command - should skip RL entry
    print("\n2. Second RL Command - Should Skip RL Entry")
    print("-" * 50)
    
    demo.simulate_command("follow_start")
    state = demo.get_state_summary()
    print(f"State after second RL command: {state}")
    
    # Demo 3: Third RL command - should still skip RL entry
    print("\n3. Third RL Command - Should Still Skip RL Entry")
    print("-" * 50)
    
    demo.simulate_command("Come to my front")
    state = demo.get_state_summary()
    print(f"State after third RL command: {state}")
    
    # Demo 4: RL interrupting command
    print("\n4. RL Interrupting Command - Should Exit RL Mode")
    print("-" * 50)
    
    demo.simulate_command("stand_down")
    state = demo.get_state_summary()
    print(f"State after interrupting command: {state}")
    
    # Demo 5: RL command after interruption - should re-enter RL mode
    print("\n5. RL Command After Interruption - Should Re-enter RL Mode")
    print("-" * 50)
    
    demo.simulate_command("Come to my left")
    state = demo.get_state_summary()
    print(f"State after RL command post-interruption: {state}")
    
    # Demo 6: Non-RL command
    print("\n6. Non-RL Command - Should Execute Directly")
    print("-" * 50)
    
    demo.simulate_command("follow_stop")
    state = demo.get_state_summary()
    print(f"State after non-RL command: {state}")
    
    print("\n" + "=" * 70)
    print("Key Improvements Demonstrated:")
    print("- RL mode entry is skipped when already in RL mode")
    print("- RL state persists across multiple RL commands")
    print("- No unnecessary RL mode transitions")
    print("- Explicit RL interruption when needed")
    print("=" * 70)


if __name__ == "__main__":
    main()
