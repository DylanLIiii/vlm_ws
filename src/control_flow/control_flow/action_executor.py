import time
import json
from enum import Enum, auto
from collections import deque

from std_msgs.msg import String
from sensor_msgs.msg import Joy


class TaskState(Enum):
    """State machine for task execution with automatic state management"""
    IDLE = auto()
    ENTERING_RL_MODE = auto()
    EXECUTING_TASK = auto()
    COMPLETING_TASK = auto()
    RECOVERING_STANCE = auto()


class ActionExecutor:
    """
    Handles the execution of mapped commands by publishing appropriate ROS2 messages.

    This class takes mapped commands from the ASRProcessor and executes them by publishing
    Joy messages for robot control and String messages for following commands.
    """

    def __init__(self, node):
        """
        Initialize the ActionExecutor.

        Args:
            node: ROS2 node instance for publishing and logging
        """
        self.node = node
        self.logger = node.get_logger()

        # Create publishers for different command types
        self.joy_publisher = self.node.create_publisher(Joy, '/joy', 10)
        self.following_publisher = self.node.create_publisher(String, '/following/task_type', 10)

        # Define command mappings for Joy messages
        self.joy_commands = {
            'stand_up': {
                'buttons': [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'stand_down': {
                'buttons': [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'shake_hand': {
                'buttons': [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            },
            'enter_rl_mode': {
                'buttons': [1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                'axes': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            }
        }

        # Define command mappings for following commands
        self.following_commands = {
            'follow_start': 'start following',
            'follow_stop': 'stop following'
        }

        # Task status monitoring setup
        self.task_status_subscriber = self.node.create_subscription(
            String, '/task_status', self.task_status_callback, 10
        )

        # Movement command tracking
        self.movement_commands = {
            'Come Here', 'Come to my front', 'Come to my behind/back',
            'Come to my left', 'Come to my right'
        }
        self.active_movement_command = None
        self.movement_command_start_time = None

        # Task completion parameters
        self.distance_threshold = 1.5  # meters - configurable threshold for task completion
        self.last_task_status = None

        # State management for automatic task sequencing
        self.current_state = TaskState.IDLE
        self.task_queue = deque()
        self.current_task_command = None
        self.state_start_time = None

        # Commands that require RL mode entry before execution
        self.rl_mode_required_commands = {
            'follow_start', 'Come Here', 'Come to my front',
            'Come to my behind/back', 'Come to my left', 'Come to my right'
        }

        # Commands that require standing recovery after execution
        self.standing_recovery_commands = {
            'follow_stop'  # Removed 'shake_hand' as it automatically returns to standing
        }

        # Movement commands that require standing recovery after completion
        self.movement_commands_with_recovery = {
            'Come Here', 'Come to my front', 'Come to my behind/back',
            'Come to my left', 'Come to my right'
        }

        # Long-term interruptible tasks that can be stopped by new commands
        self.interruptible_tasks = {
            'follow_start', 'Come Here', 'Come to my front',
            'Come to my behind/back', 'Come to my left', 'Come to my right'
        }

        # Short atomic actions that must complete without interruption
        self.non_interruptible_tasks = {
            'stand_up', 'stand_down', 'enter_rl_mode', 'shake_hand', 'follow_stop'
        }

        # State transition timeouts (in seconds)
        self.rl_mode_timeout = 3.0
        self.task_execution_timeout = 2.0
        self.recovery_timeout = 3.0

        self.logger.info("ActionExecutor initialized with automatic state management and task interruption")
        self.logger.info(f"Joy commands available: {list(self.joy_commands.keys())}")
        self.logger.info(f"Following commands available: {list(self.following_commands.keys())}")
        self.logger.info(f"Movement commands monitored: {list(self.movement_commands)}")
        self.logger.info(f"RL mode required for: {list(self.rl_mode_required_commands)}")
        self.logger.info(f"Standing recovery after: {list(self.standing_recovery_commands)}")
        self.logger.info(f"Interruptible tasks: {list(self.interruptible_tasks)}")
        self.logger.info(f"Non-interruptible tasks: {list(self.non_interruptible_tasks)}")
        self.logger.info(f"Task status monitoring enabled on /task_status topic")

    def execute_action(self, mapped_command):
        """
        Execute an action with automatic state management and task interruption.

        This method implements a state machine that automatically handles:
        1. Task interruption for long-term interruptible tasks
        2. Pre-task RL mode entry for navigation commands
        3. Task execution
        4. Post-task standing recovery

        Args:
            mapped_command (str): The mapped command from ASRProcessor

        Returns:
            bool: True if action was initiated successfully, False otherwise
        """
        if not isinstance(mapped_command, str):
            self.logger.error(f"Invalid mapped command type: {type(mapped_command)}")
            return False

        # Handle system state and interruption logic
        if self.current_state != TaskState.IDLE:
            return self._handle_busy_state(mapped_command)

        # Start the state machine for this command
        return self._start_task_sequence(mapped_command)

    def _handle_busy_state(self, new_command):
        """
        Handle incoming commands when the system is busy.

        Args:
            new_command (str): The new command to execute

        Returns:
            bool: True if command was handled successfully, False otherwise
        """
        current_task = self.current_task_command

        # Check if current task is interruptible
        if current_task and current_task in self.interruptible_tasks:
            self.logger.info(f"Interrupting interruptible task '{current_task}' for new command '{new_command}'")
            self._interrupt_current_task()
            return self._start_task_sequence(new_command)

        # Current task is non-interruptible, check if it's a critical state
        elif self.current_state in [TaskState.ENTERING_RL_MODE, TaskState.RECOVERING_STANCE]:
            # These are short transitions, queue the command
            self.logger.info(f"System in critical state ({self.current_state.name}), queueing command: '{new_command}'")
            self.task_queue.append(new_command)
            return True

        # Current task is non-interruptible and executing
        else:
            self.logger.info(f"Non-interruptible task '{current_task}' executing, queueing command: '{new_command}'")
            self.task_queue.append(new_command)
            return True

    def _interrupt_current_task(self):
        """
        Interrupt the currently executing task.

        This method handles stopping interruptible tasks and cleaning up state.
        """
        current_task = self.current_task_command

        if not current_task:
            return

        self.logger.info(f"Interrupting task: '{current_task}'")

        # Handle specific interruption logic based on task type
        if current_task == 'follow_start':
            # Stop following immediately
            self._publish_following_command('follow_stop')
            self.logger.info("Sent stop following command due to task interruption")

        elif current_task in self.movement_commands:
            # Stop movement command tracking and send stop following
            if self.active_movement_command:
                self._stop_movement_command_tracking()
                self._publish_following_command('follow_stop')
                self.logger.info(f"Stopped movement tracking for '{current_task}' due to interruption")

        # Reset state to idle for immediate new task execution
        self.current_state = TaskState.IDLE
        self.current_task_command = None
        self.state_start_time = None

    def _start_task_sequence(self, command):
        """
        Start the task sequence for a given command with appropriate state transitions.

        Args:
            command (str): The command to execute

        Returns:
            bool: True if sequence started successfully, False otherwise
        """
        self.current_task_command = command
        self.logger.info(f"Starting task sequence for command: '{command}'")

        # Determine if this command requires RL mode entry
        if command in self.rl_mode_required_commands:
            self.logger.info(f"Command '{command}' requires RL mode - entering RL mode first")
            return self._transition_to_state(TaskState.ENTERING_RL_MODE)
        else:
            # Direct execution for commands that don't need RL mode
            self.logger.info(f"Command '{command}' executing directly")
            return self._transition_to_state(TaskState.EXECUTING_TASK)

    def _transition_to_state(self, new_state):
        """
        Transition to a new state and execute the appropriate action.

        Args:
            new_state (TaskState): The state to transition to

        Returns:
            bool: True if transition was successful, False otherwise
        """
        old_state = self.current_state
        self.current_state = new_state
        self.state_start_time = time.time()

        self.logger.info(f"State transition: {old_state.name} -> {new_state.name}")

        # Execute the action for the new state
        if new_state == TaskState.ENTERING_RL_MODE:
            return self._execute_rl_mode_entry()
        elif new_state == TaskState.EXECUTING_TASK:
            return self._execute_current_task()
        elif new_state == TaskState.RECOVERING_STANCE:
            return self._execute_standing_recovery()
        elif new_state == TaskState.IDLE:
            return self._execute_idle_transition()
        else:
            self.logger.error(f"Unknown state transition: {new_state}")
            return False

    def _execute_rl_mode_entry(self):
        """Execute RL mode entry and set up transition to task execution."""
        success = self._publish_joy_command('enter_rl_mode')
        if success:
            self.logger.info("RL mode entry initiated - will transition to task execution")
            # Schedule transition to task execution after a delay
            self.node.create_timer(self.rl_mode_timeout, self._on_rl_mode_complete)
        return success

    def _execute_current_task(self):
        """Execute the current task command."""
        command = self.current_task_command

        # Check if it's a Joy command
        if command in self.joy_commands:
            success = self._publish_joy_command(command)
        # Check if it's a following command
        elif command in self.following_commands:
            success = self._publish_following_command(command)
        # Handle movement commands
        elif command in self.movement_commands:
            self._start_movement_command_tracking(command)
            success = True
            self.logger.info(f"Movement command '{command}' will be handled by task logic - tracking started")
        else:
            success = True
            self.logger.info(f"Command '{command}' will be handled by task logic")

        if success:
            # Determine next state based on command type
            if command in self.standing_recovery_commands:
                # Commands that need immediate standing recovery (only follow_stop now)
                self.logger.info(f"Command '{command}' requires standing recovery")
                self.node.create_timer(self.task_execution_timeout, self._on_task_complete_with_recovery)
            elif command in self.movement_commands_with_recovery:
                # Movement commands - recovery will be triggered by task completion monitoring
                self.logger.info(f"Movement command '{command}' - waiting for task completion")
            elif command == 'shake_hand':
                # Shake hand automatically returns to standing, no recovery needed
                self.logger.info(f"Shake hand command executed - robot will automatically return to standing")
                self.node.create_timer(self.task_execution_timeout, self._on_task_complete_no_recovery)
            else:
                # Commands that don't need recovery - go back to idle
                self.node.create_timer(self.task_execution_timeout, self._on_task_complete_no_recovery)

        return success

    def _execute_standing_recovery(self):
        """Execute standing recovery and transition back to idle."""
        success = self._publish_joy_command('stand_up')
        if success:
            self.logger.info("Standing recovery initiated - will transition to idle")
            self.node.create_timer(self.recovery_timeout, self._on_recovery_complete)
        return success

    def _execute_idle_transition(self):
        """Handle transition to idle state and process any queued commands."""
        self.current_task_command = None
        self.logger.info("Returned to idle state")

        # Process next command in queue if any
        if self.task_queue:
            next_command = self.task_queue.popleft()
            self.logger.info(f"Processing queued command: '{next_command}'")
            return self._start_task_sequence(next_command)

        return True

    def _on_rl_mode_complete(self):
        """Timer callback for RL mode completion."""
        if self.current_state == TaskState.ENTERING_RL_MODE:
            self.logger.info("RL mode entry timeout reached - transitioning to task execution")
            self._transition_to_state(TaskState.EXECUTING_TASK)

    def _on_task_complete_with_recovery(self):
        """Timer callback for task completion that requires standing recovery."""
        if self.current_state == TaskState.EXECUTING_TASK:
            self.logger.info("Task execution timeout reached - transitioning to standing recovery")
            self._transition_to_state(TaskState.RECOVERING_STANCE)

    def _on_task_complete_no_recovery(self):
        """Timer callback for task completion that doesn't require recovery."""
        if self.current_state == TaskState.EXECUTING_TASK:
            self.logger.info("Task execution timeout reached - transitioning to idle")
            self._transition_to_state(TaskState.IDLE)

    def _on_recovery_complete(self):
        """Timer callback for standing recovery completion."""
        if self.current_state == TaskState.RECOVERING_STANCE:
            self.logger.info("Standing recovery timeout reached - transitioning to idle")
            self._transition_to_state(TaskState.IDLE)

    def _publish_joy_command(self, command):
        """
        Publish a Joy message for robot control commands.

        Args:
            command (str): The command to execute

        Returns:
            bool: True if published successfully, False otherwise
        """
        try:
            if command not in self.joy_commands:
                self.logger.error(f"Unknown Joy command: {command}")
                return False

            # Create Joy message
            joy_msg = Joy()
            joy_msg.header.stamp = self.node.get_clock().now().to_msg()
            joy_msg.header.frame_id = "joy"

            # Set buttons and axes from mapping
            joy_msg.buttons = self.joy_commands[command]['buttons']
            joy_msg.axes = self.joy_commands[command]['axes']

            # Publish the message
            self.joy_publisher.publish(joy_msg)
            self.logger.info(f"Published Joy command for '{command}': buttons={joy_msg.buttons}, axes={joy_msg.axes}")
            return True

        except Exception as e:
            self.logger.error(f"Error publishing Joy command '{command}': {e}")
            return False

    def _publish_following_command(self, command):
        """
        Publish a String message for following commands.

        Args:
            command (str): The command to execute

        Returns:
            bool: True if published successfully, False otherwise
        """
        try:
            if command not in self.following_commands:
                self.logger.error(f"Unknown following command: {command}")
                return False

            # Create String message
            string_msg = String()
            string_msg.data = self.following_commands[command]

            # Publish the message
            self.following_publisher.publish(string_msg)
            self.logger.info(f"Published following command for '{command}': '{string_msg.data}'")
            return True

        except Exception as e:
            self.logger.error(f"Error publishing following command '{command}': {e}")
            return False

    def enter_rl_mode(self):
        """
        Special method to enter RL mode by publishing the appropriate Joy command.

        Returns:
            bool: True if published successfully, False otherwise
        """
        return self._publish_joy_command('enter_rl_mode')

    def _start_movement_command_tracking(self, command):
        """
        Start tracking a movement command for completion monitoring.

        Args:
            command (str): The movement command to track
        """
        self.active_movement_command = command
        self.movement_command_start_time = time.time()
        self.logger.info(f"Started tracking movement command: '{command}'")

    def _stop_movement_command_tracking(self):
        """
        Stop tracking the current movement command.
        """
        if self.active_movement_command:
            duration = time.time() - self.movement_command_start_time if self.movement_command_start_time else 0
            self.logger.info(f"Stopped tracking movement command: '{self.active_movement_command}' (duration: {duration:.1f}s)")
            self.active_movement_command = None
            self.movement_command_start_time = None

    def task_status_callback(self, msg):
        """
        Callback for task status updates from /task_status topic.

        Args:
            msg (String): JSON-formatted string containing task status information
        """
        try:
            # Parse JSON from the message
            task_status = json.loads(msg.data)
            self.last_task_status = task_status

            # Check if we're currently tracking a movement command
            if self.active_movement_command:
                self._check_movement_task_completion(task_status)

        except json.JSONDecodeError as e:
            self.logger.error(f"Failed to parse task status JSON: {e}")
        except Exception as e:
            self.logger.error(f"Error processing task status: {e}")

    def _check_movement_task_completion(self, task_status):
        """
        Check if the current movement task is complete based on task status.

        Args:
            task_status (dict): Parsed task status information
        """
        try:
            # Extract required fields from task status
            current_distance = task_status.get('current_distance_to_target')
            target_reached = task_status.get('target_reached', False)

            # Log task status for debugging
            if current_distance is not None:
                self.logger.debug(f"Task status - Distance: {current_distance:.2f}m, Target reached: {target_reached}")

            # Check completion conditions
            task_complete = False
            completion_reason = ""

            if target_reached:
                task_complete = True
                completion_reason = "target_reached flag is True"
            elif current_distance is not None and current_distance <= self.distance_threshold:
                task_complete = True
                completion_reason = f"distance ({current_distance:.2f}m) below threshold ({self.distance_threshold}m)"

            if task_complete:
                self.logger.info(f"Movement task '{self.active_movement_command}' completed: {completion_reason}")
                self._handle_movement_task_completion()

        except Exception as e:
            self.logger.error(f"Error checking movement task completion: {e}")

    def _handle_movement_task_completion(self):
        """
        Handle the completion of a movement task with automatic state management.
        """
        try:
            # Stop tracking the movement command
            completed_command = self.active_movement_command
            self._stop_movement_command_tracking()

            # Automatically publish stop following command
            stop_success = self._publish_following_command('follow_stop')

            if stop_success:
                self.logger.info(f"Automatically stopped following after completing movement task: '{completed_command}'")

                # If this was a movement command that requires standing recovery, transition to recovery
                if completed_command in self.movement_commands_with_recovery and self.current_state == TaskState.EXECUTING_TASK:
                    self.logger.info(f"Movement task '{completed_command}' completed - transitioning to standing recovery")
                    self._transition_to_state(TaskState.RECOVERING_STANCE)
                else:
                    self.logger.info(f"Movement task '{completed_command}' completed - transitioning to idle")
                    self._transition_to_state(TaskState.IDLE)
            else:
                self.logger.warn(f"Failed to automatically stop following after completing movement task: '{completed_command}'")
                # Even if stop following failed, still transition to recovery/idle
                if completed_command in self.movement_commands_with_recovery and self.current_state == TaskState.EXECUTING_TASK:
                    self._transition_to_state(TaskState.RECOVERING_STANCE)
                else:
                    self._transition_to_state(TaskState.IDLE)

        except Exception as e:
            self.logger.error(f"Error handling movement task completion: {e}")

    def get_task_status(self):
        """
        Get the latest task status information.

        Returns:
            dict: Latest task status or None if no status received
        """
        return self.last_task_status

    def is_movement_command_active(self):
        """
        Check if a movement command is currently being tracked.

        Returns:
            bool: True if a movement command is active, False otherwise
        """
        return self.active_movement_command is not None

    def get_active_movement_command(self):
        """
        Get the currently active movement command.

        Returns:
            str: Active movement command or None if no command is active
        """
        return self.active_movement_command

    def set_distance_threshold(self, threshold):
        """
        Set the distance threshold for task completion detection.

        Args:
            threshold (float): Distance threshold in meters
        """
        if threshold > 0:
            self.distance_threshold = threshold
            self.logger.info(f"Distance threshold updated to {threshold}m")
        else:
            self.logger.error(f"Invalid distance threshold: {threshold}. Must be positive.")

    def get_current_state(self):
        """
        Get the current state of the action executor.

        Returns:
            TaskState: Current state of the state machine
        """
        return self.current_state

    def is_busy(self):
        """
        Check if the action executor is currently busy executing a task.

        Returns:
            bool: True if busy (not in IDLE state), False if idle
        """
        return self.current_state != TaskState.IDLE

    def is_current_task_interruptible(self):
        """
        Check if the currently executing task can be interrupted.

        Returns:
            bool: True if current task is interruptible, False otherwise
        """
        if self.current_state == TaskState.IDLE:
            return True  # No task running, can accept new commands

        current_task = self.current_task_command
        return current_task and current_task in self.interruptible_tasks

    def get_current_task_command(self):
        """
        Get the currently executing task command.

        Returns:
            str: Current task command or None if idle
        """
        return self.current_task_command

    def get_queued_commands_count(self):
        """
        Get the number of commands currently queued for execution.

        Returns:
            int: Number of queued commands
        """
        return len(self.task_queue)

    def clear_command_queue(self):
        """
        Clear all queued commands. Use with caution.
        """
        cleared_count = len(self.task_queue)
        self.task_queue.clear()
        if cleared_count > 0:
            self.logger.warn(f"Cleared {cleared_count} queued commands")

    def get_state_duration(self):
        """
        Get how long the current state has been active.

        Returns:
            float: Duration in seconds, or None if no state start time
        """
        if self.state_start_time is None:
            return None
        return time.time() - self.state_start_time

    def force_idle_state(self):
        """
        Force transition to idle state. Use only for emergency situations.
        This will interrupt any current task and clear the command queue.
        """
        current_task = self.current_task_command
        if current_task:
            self.logger.warn(f"Emergency stop: Forcing transition to IDLE state, interrupting '{current_task}'")

            # If it was a following or movement task, send stop command
            if current_task in self.interruptible_tasks:
                self._publish_following_command('follow_stop')
        else:
            self.logger.warn("Forcing transition to IDLE state")

        self.current_state = TaskState.IDLE
        self.current_task_command = None
        self.state_start_time = None
        self._stop_movement_command_tracking()
        self.clear_command_queue()
