#!/usr/bin/env python3
"""
ASR Protection Feature Demonstration

This script demonstrates how the ASR Protection feature works in the TaskLogicNode.
It shows the buffering, deduplication, and sequential execution of ASR commands.

Usage:
    python3 asr_protection_demo.py

The script will simulate ASR commands being received and show how they are processed
with and without ASR protection enabled.
"""

import time
import threading


class ASRProtectionDemo:
    """Simplified demonstration of ASR Protection logic"""
    
    def __init__(self):
        # ASR Protection settings
        self.asr_protection_enabled = True
        self.asr_command_buffer = []  # List to maintain exact order and allow non-consecutive duplicates
        self.asr_buffer_timer = None
        self.asr_buffer_lock = threading.Lock()
        self.last_asr_activity_time = None
        self.asr_buffering_window = 1.0  # 1 second
        self.asr_inactivity_threshold = 1.0  # 1 second
        
        # Command mapping (simplified)
        self.command_mappings = {
            'move_to_master': 'Come Here',
            'move_to_master_front': 'Come to my front',
            'follow_start': 'follow_start',
            'follow_stop': 'follow_stop',
            'action_stand': 'stand_up',
            'action_sit': 'stand_down'
        }
        
    def log(self, message):
        """Simple logging function"""
        timestamp = time.strftime("%H:%M:%S", time.localtime())
        print(f"[{timestamp}] {message}")
        
    def simulate_asr_command(self, command):
        """Simulate receiving an ASR command"""
        self.log(f"ASR command received: '{command}'")
        
        if self.asr_protection_enabled:
            self._handle_asr_command_with_protection(command)
        else:
            self._process_asr_command_immediately(command)
            
    def _handle_asr_command_with_protection(self, asr_command):
        """Handle ASR command with buffering and consecutive duplicate removal"""
        with self.asr_buffer_lock:
            current_time = time.time()

            # Check if this is the first command after inactivity
            if (self.last_asr_activity_time is None or
                current_time - self.last_asr_activity_time > self.asr_inactivity_threshold):

                # Start buffering window
                self.log(f"Starting ASR buffering window - first command after inactivity: '{asr_command}'")
                self._start_asr_buffering_window()

            # Update last activity time
            self.last_asr_activity_time = current_time

            # Add command to buffer only if it's not the same as the last command (consecutive duplicate removal)
            if not self.asr_command_buffer or self.asr_command_buffer[-1] != asr_command:
                self.asr_command_buffer.append(asr_command)
                self.log(f"Added ASR command to buffer: '{asr_command}' (buffer size: {len(self.asr_command_buffer)})")
            else:
                self.log(f"Consecutive duplicate ASR command ignored: '{asr_command}'")
                
    def _start_asr_buffering_window(self):
        """Start the ASR buffering window timer"""
        # Cancel any existing timer
        if self.asr_buffer_timer is not None:
            self.asr_buffer_timer.cancel()
        
        # Start new timer for buffering window
        self.asr_buffer_timer = threading.Timer(self.asr_buffering_window, self._execute_buffered_asr_commands)
        self.asr_buffer_timer.start()
        self.log(f"ASR buffering window started - will execute commands in {self.asr_buffering_window}s")
        
    def _execute_buffered_asr_commands(self):
        """Execute all buffered ASR commands sequentially"""
        with self.asr_buffer_lock:
            if not self.asr_command_buffer:
                self.log("ASR buffering window ended - no commands to execute")
                return

            commands_to_execute = self.asr_command_buffer.copy()
            self.log(f"ASR buffering window ended - executing {len(commands_to_execute)} commands: {commands_to_execute}")

            # Clear the buffer
            self.asr_command_buffer.clear()
            self.asr_buffer_timer = None
        
        # Execute commands sequentially (outside the lock to avoid blocking)
        for command in commands_to_execute:
            self.log(f"Executing buffered ASR command: '{command}'")
            self._process_asr_command_immediately(command)
            
    def _process_asr_command_immediately(self, asr_command):
        """Process ASR command immediately without buffering"""
        # Map the command
        mapped_command = self.command_mappings.get(asr_command, None)
        
        if mapped_command:
            self.log(f"ASR command mapped: '{asr_command}' -> '{mapped_command}'")
            self.log(f"Executing action for: '{mapped_command}'")
        else:
            self.log(f"ASR command could not be mapped: '{asr_command}'")


def main():
    """Main demonstration function"""
    demo = ASRProtectionDemo()
    
    print("=" * 60)
    print("ASR Protection Feature Demonstration")
    print("=" * 60)
    
    # Demo 1: ASR Protection Enabled (default)
    print("\n1. ASR Protection ENABLED - Buffering and Consecutive Duplicate Removal")
    print("-" * 70)

    demo.asr_protection_enabled = True

    # Simulate rapid ASR commands with consecutive and non-consecutive duplicates
    demo.simulate_asr_command("move_to_master")
    time.sleep(0.1)
    demo.simulate_asr_command("move_to_master")  # Consecutive duplicate - should be ignored
    time.sleep(0.1)
    demo.simulate_asr_command("follow_start")
    time.sleep(0.1)
    demo.simulate_asr_command("action_stand")
    time.sleep(0.1)
    demo.simulate_asr_command("move_to_master")  # Non-consecutive duplicate - should be kept
    time.sleep(0.1)
    demo.simulate_asr_command("move_to_master")  # Consecutive duplicate - should be ignored
    
    # Wait for buffering window to complete
    print("\nWaiting for buffering window to complete...")
    time.sleep(1.5)
    
    # Demo 2: ASR Protection Disabled
    print("\n\n2. ASR Protection DISABLED - Immediate Processing")
    print("-" * 50)
    
    demo.asr_protection_enabled = False
    
    # Simulate the same commands
    demo.simulate_asr_command("move_to_master")
    time.sleep(0.1)
    demo.simulate_asr_command("follow_start")
    time.sleep(0.1)
    demo.simulate_asr_command("move_to_master")  # Will be processed again
    time.sleep(0.1)
    demo.simulate_asr_command("action_stand")
    time.sleep(0.1)
    demo.simulate_asr_command("follow_start")  # Will be processed again
    
    # Demo 3: Inactivity Reset
    print("\n\n3. Inactivity Reset - New Buffering Window")
    print("-" * 50)
    
    demo.asr_protection_enabled = True
    
    demo.simulate_asr_command("move_to_master")
    time.sleep(0.2)
    demo.simulate_asr_command("follow_start")
    
    # Wait for buffering window to complete
    time.sleep(1.2)
    
    # After inactivity, new commands should start a new buffering window
    print("\nAfter inactivity period, sending new commands...")
    demo.simulate_asr_command("action_sit")
    time.sleep(0.1)
    demo.simulate_asr_command("follow_stop")
    
    # Wait for final buffering window
    time.sleep(1.5)
    
    print("\n" + "=" * 60)
    print("Demonstration completed!")
    print("=" * 60)


if __name__ == "__main__":
    main()
