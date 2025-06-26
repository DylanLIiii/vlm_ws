import time
import json
import threading
from enum import Enum, auto
from collections import deque

from std_msgs.msg import String
from sensor_msgs.msg import Joy


class TaskState(Enum):
    """State machine for task execution with persistent RL state management"""
    IDLE = auto()
    RL_MODE = auto()
    EXECUTING_TASK = auto()
    COMPLETING_TASK = auto()
    RECOVERING_STANCE = auto()


class RLState(Enum):
    """Robot's current operational mode state"""
    RL_MODE = auto()      # Robot is in reinforcement learning mode
    STANDING = auto()     # Robot is standing (non-RL mode)
    SITTING = auto()      # Robot is sitting down (non-RL mode)
    SHAKING_HANDS = auto() # Robot is performing handshake (non-RL mode)


class ActionExecutor:
    """
    Handles the execution of mapped commands with persistent RL state management.

    This class takes mapped commands from the ASRProcessor and executes them by publishing
    Joy messages for robot control and String messages for following commands. It implements
    persistent RL state management where the robot maintains its RL mode continuously instead
    of entering/exiting RL mode for each task, only switching states when necessary for
    specific interrupting actions (stand_up, shake_hand, stand_down).
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
        self.following_publisher = self.node.create_publisher(String, '/following_control', 10)

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
            String, '/task_status', self.task_status_callback, 1
        )

        # Movement command tracking
        self.movement_commands = {
            'Come Here', 'Come to my front', 'Come to my behind/back',
            'Come to my left', 'Come to my right'
        }
        self.active_movement_command = None
        self.movement_command_start_time = None

        # Follow command tracking
        self.active_follow_command = None
        self.follow_command_start_time = None

        # Countdown management to prevent multiple countdowns from running
        self.active_countdown_thread = None
        self.countdown_cancelled = False

        # Task completion parameters
        self.distance_threshold = 1.5  # meters - configurable threshold for task completion
        self.last_task_status = None

        # State management for automatic task sequencing
        self.current_state = TaskState.IDLE
        self.task_queue = deque()
        self.current_task_command = None
        self.state_start_time = None

        # Persistent RL state management
        self.current_rl_state = RLState.STANDING  # Robot starts in standing mode
        self.rl_state_initialized = False  # Track if RL mode has been initialized

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

        # Commands that require standing recovery after execution (deprecated - will be removed)
        self.standing_recovery_commands = {
            # follow_stop removed - it should transition to idle, not require standing recovery
        }

        # RL-based commands that stay in RL mode after completion (no standing recovery)
        self.rl_commands_stay_in_rl = {
            'follow_start', 'Come Here', 'Come to my front', 'Come to my behind/back',
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

        # Commands with explicit status monitoring (completion determined via /task_status topic)
        self.status_monitored_commands = {
            'follow_start',  # Monitored via task status updates
            'Come Here', 'Come to my front', 'Come to my behind/back',
            'Come to my left', 'Come to my right'  # Monitored via movement tracking
        }

        # Commands without status monitoring (rely on timeout-based completion)
        self.timeout_based_commands = {
            'stand_up', 'stand_down', 'shake_hand', 'enter_rl_mode', 'follow_stop'
        }

        # State transition timeouts (in seconds)
        # To check after how long we should determine the no feedback task has been completed.
        self.rl_mode_timeout = 1.5
        self.task_execution_timeout = 5.0
        self.recovery_timeout = 3.0

        self.logger.info("ActionExecutor initialized with persistent RL state management and task interruption")
        self.logger.info(f"Initial RL state: {self.current_rl_state.name}")
        self.logger.info(f"Joy commands available: {list(self.joy_commands.keys())}")
        self.logger.info(f"Following commands available: {list(self.following_commands.keys())}")
        self.logger.info(f"Movement commands monitored: {list(self.movement_commands)}")
        self.logger.info(f"RL mode required for: {list(self.rl_mode_required_commands)}")
        self.logger.info(f"RL interrupting commands: {list(self.rl_interrupting_commands.keys())}")
        self.logger.info(f"RL commands (stay in RL mode): {list(self.rl_commands_stay_in_rl)}")
        self.logger.info(f"Interruptible tasks: {list(self.interruptible_tasks)}")
        self.logger.info(f"Non-interruptible tasks: {list(self.non_interruptible_tasks)}")
        self.logger.info(f"Status monitored commands: {list(self.status_monitored_commands)}")
        self.logger.info(f"Timeout-based commands: {list(self.timeout_based_commands)}")
        self.logger.info(f"Task status monitoring enabled on /task_status topic")

    def execute_action(self, mapped_command):
        """
        Execute an action with persistent RL state management and task interruption.

        This method implements a state machine that automatically handles:
        1. Task interruption for long-term interruptible tasks
        2. Persistent RL mode management (enter RL mode only when needed, stay in RL mode)
        3. RL mode interruption for specific commands (stand_up, shake_hand, stand_down)
        4. Task execution with state awareness
        5. Proper state transitions based on current RL state

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
        elif self.current_state in [TaskState.RL_MODE, TaskState.RECOVERING_STANCE]:
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
            # Stop following immediately and stop tracking
            self._publish_following_command('follow_stop')
            if self.active_follow_command:
                self._stop_follow_command_tracking()
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
        Start the task sequence for a given command with persistent RL state management.

        Args:
            command (str): The command to execute

        Returns:
            bool: True if sequence started successfully, False otherwise
        """
        self.current_task_command = command
        self.logger.info(f"Starting task sequence for command: '{command}' (current RL state: {self.current_rl_state.name})")

        # Handle RL interrupting commands (stand_up, shake_hand, stand_down)
        if command in self.rl_interrupting_commands:
            target_rl_state = self.rl_interrupting_commands[command]
            self.logger.info(f"Command '{command}' interrupts RL mode - transitioning to {target_rl_state.name}")
            self.current_rl_state = target_rl_state
            return self._transition_to_state(TaskState.EXECUTING_TASK)

        # Handle commands that require RL mode
        elif command in self.rl_mode_required_commands:
            if self.should_skip_rl_entry(command):
                self.log_rl_state_transition(command, "Skipping RL entry - already in RL mode")
                return self._transition_to_state(TaskState.EXECUTING_TASK)
            else:
                self.log_rl_state_transition(command, f"Entering RL mode from {self.current_rl_state.name}")
                self.current_rl_state = RLState.RL_MODE
                return self._transition_to_state(TaskState.RL_MODE)

        # Handle other commands (follow_stop, etc.)
        else:
            self.logger.info(f"Command '{command}' executing directly in current state")
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
        if new_state == TaskState.RL_MODE:
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
            self._create_countdown(self.rl_mode_timeout, self._on_rl_mode_complete)
        return success

    def _execute_current_task(self):
        """Execute the current task command."""
        command = self.current_task_command

        # Handle case where task command is None (task already completed)
        if command is None:
            self.logger.info("No current task command - transitioning to idle")
            self._transition_to_state(TaskState.IDLE)
            return True

        # Check if it's a Joy command
        if command in self.joy_commands:
            success = self._publish_joy_command(command)
        # Check if it's a following command
        elif command in self.following_commands:
            success = self._publish_following_command(command)
            # Start tracking for follow_start command
            if command == 'follow_start':
                self._start_follow_command_tracking(command)
                self.logger.info(f"Follow command '{command}' tracking started")
        # Handle movement commands
        elif command in self.movement_commands:
            self._start_movement_command_tracking(command)
            success = True
            self.logger.info(f"Movement command '{command}' will be handled by task logic - tracking started")
        else:
            success = True
            self.logger.info(f"Command '{command}' will be handled by task logic")
        if success:
            # Determine next state based on command type and monitoring capabilities
            if command in self.status_monitored_commands:
                # Commands with explicit status monitoring - wait for completion via /task_status topic
                self._cancel_active_countdown()
                self.logger.info(f"Status-monitored command '{command}' - waiting for explicit completion signal")  
                # No timeout timer or ignore previout timer - completion will be handled by task status callback
                
            elif command == 'shake_hand':
                # Shake hand automatically returns to standing, no recovery needed
                self.logger.info(f"Shake hand command executed - robot will automatically return to standing")
                self._create_countdown(self.task_execution_timeout, self._on_task_complete_no_recovery)
            elif command in self.rl_interrupting_commands:
                # RL interrupting commands (stand_up, stand_down) - transition to idle after completion
                self.logger.info(f"RL interrupting command '{command}' executed - will transition to idle")
                self._create_countdown(self.task_execution_timeout, self._on_task_complete_no_recovery)
            elif command in self.timeout_based_commands:
                # Commands without status monitoring - use timeout-based completion
                if command in self.rl_mode_required_commands:
                    self.logger.info(f"Timeout-based RL command '{command}' - will stay in RL mode after timeout")
                    self._create_countdown(self.task_execution_timeout, self._on_task_complete_stay_rl)
                elif command == 'follow_stop':
                    # follow_stop should always transition to idle, regardless of current RL state
                    self.logger.info(f"Follow stop command '{command}' - will transition to idle after timeout")
                    self._create_countdown(self.task_execution_timeout, self._on_task_complete_no_recovery)
                elif command in self.standing_recovery_commands:
                    # Commands that require standing recovery
                    self.logger.info(f"Timeout-based command '{command}' - will transition to standing recovery after timeout")
                    self._create_countdown(self.task_execution_timeout, self._on_task_complete_with_recovery)
                else:
                    self.logger.info(f"Timeout-based command '{command}' - will transition based on RL state after timeout")
                    if self.current_rl_state == RLState.RL_MODE:
                        self._create_countdown(self.task_execution_timeout, self._on_task_complete_stay_rl)
                    else:
                        self._create_countdown(self.task_execution_timeout, self._on_task_complete_no_recovery)
            else:
                # Fallback for other commands - go back to appropriate state based on RL state
                self.logger.info(f"Fallback handling for command '{command}' - using timeout-based completion")
                if self.current_rl_state == RLState.RL_MODE:
                    self.logger.info(f"Command '{command}' completed - staying in RL mode")
                    self._create_countdown(self.task_execution_timeout, self._on_task_complete_stay_rl)
                else:
                    self.logger.info(f"Command '{command}' completed - returning to idle")
                    self._create_countdown(self.task_execution_timeout, self._on_task_complete_no_recovery)

        return success

    def _execute_standing_recovery(self):
        """Execute standing recovery and transition back to idle."""
        success = self._publish_joy_command('stand_up')
        if success:
            self.logger.info("Standing recovery initiated - will transition to idle")
            self._create_countdown(self.recovery_timeout, self._on_recovery_complete)
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
        """Countdown callback for RL mode completion."""
        self.active_countdown_thread = None  # Clear countdown reference
        if self.current_state == TaskState.RL_MODE:
            self.logger.info("RL mode entry timeout reached - transitioning to task execution")
            self._transition_to_state(TaskState.EXECUTING_TASK)

    def _on_task_complete_stay_rl(self):
        """Countdown callback for task completion that stays in RL mode."""
        self.active_countdown_thread = None  # Clear countdown reference
        if self.current_state == TaskState.EXECUTING_TASK:
            self.logger.info("Task execution timeout reached - staying in RL mode, transitioning to idle")
            # Maintain RL state but transition to idle (don't re-enter RL mode)
            self._transition_to_state(TaskState.IDLE)

    def _on_task_complete_with_recovery(self):
        """Countdown callback for task completion that requires standing recovery."""
        self.active_countdown_thread = None  # Clear countdown reference
        if self.current_state == TaskState.EXECUTING_TASK:
            self.logger.info("Task execution timeout reached - transitioning to standing recovery")
            self._transition_to_state(TaskState.RECOVERING_STANCE)

    def _on_task_complete_no_recovery(self):
        """Countdown callback for task completion that doesn't require recovery."""
        self.active_countdown_thread = None  # Clear countdown reference
        if self.current_state == TaskState.EXECUTING_TASK:
            self.logger.info("Task execution timeout reached - transitioning to idle")
            self._transition_to_state(TaskState.IDLE)

    def _on_recovery_complete(self):
        """Countdown callback for standing recovery completion."""
        self.active_countdown_thread = None  # Clear countdown reference
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

    def exit_rl_mode(self, target_state=RLState.STANDING):
        """
        Explicitly exit RL mode and transition to a non-RL state.

        Args:
            target_state (RLState): The target non-RL state to transition to

        Returns:
            bool: True if state was changed, False otherwise
        """
        if self.current_rl_state == RLState.RL_MODE:
            old_state = self.current_rl_state
            self.current_rl_state = target_state
            self.logger.info(f"Explicitly exiting RL mode: {old_state.name} -> {target_state.name}")
            return True
        else:
            self.logger.info(f"Already in non-RL state: {self.current_rl_state.name}")
            return False

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

    def _start_follow_command_tracking(self, command):
        """
        Start tracking a follow command for completion monitoring.

        Args:
            command (str): The follow command to track
        """
        self.active_follow_command = command
        self.follow_command_start_time = time.time()
        self.logger.info(f"Started tracking follow command: '{command}'")

    def _stop_follow_command_tracking(self):
        """
        Stop tracking the current follow command.
        """
        if self.active_follow_command:
            duration = time.time() - self.follow_command_start_time if self.follow_command_start_time else 0
            self.logger.info(f"Stopped tracking follow command: '{self.active_follow_command}' (duration: {duration:.1f}s)")
            self.active_follow_command = None
            self.follow_command_start_time = None

    def _cancel_active_countdown(self):
        """
        Cancel the currently active countdown to prevent multiple countdowns from running.
        """
        if self.active_countdown_thread is not None:
            try:
                self.countdown_cancelled = True
                self.logger.debug("Cancelled previous countdown")
            except Exception as e:
                self.logger.debug(f"Error cancelling countdown: {e}")
            finally:
                self.active_countdown_thread = None

    def _countdown_worker(self, timeout, callback):
        """
        Worker function that performs the countdown in a separate thread.

        Args:
            timeout (float): Countdown timeout in seconds
            callback: Callback function to execute when countdown completes
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.countdown_cancelled:
                self.logger.debug("Countdown was cancelled")
                return
            time.sleep(0.1)  # Check every 100ms for cancellation

        # If we reach here, countdown completed without cancellation
        if not self.countdown_cancelled:
            callback()

    def _create_countdown(self, timeout, callback):
        """
        Create a new countdown, cancelling any existing countdown first.

        Args:
            timeout (float): Countdown timeout in seconds
            callback: Callback function to execute when countdown completes
        """
        self._cancel_active_countdown()
        self.countdown_cancelled = False
        self.active_countdown_thread = threading.Thread(
            target=self._countdown_worker,
            args=(timeout, callback),
            daemon=True
        )
        self.active_countdown_thread.start()
        return self.active_countdown_thread

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

            # Check if we're currently tracking a follow command
            if self.active_follow_command:
                self._check_follow_task_completion(task_status)

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
            current_distance = task_status.get('distance_to_target')
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

    def _check_follow_task_completion(self, task_status):
        """
        Check if the current follow task is complete based on task status.

        Args:
            task_status (dict): Parsed task status information
        """
        try:
            # For follow commands, we can use different completion criteria
            # For now, we'll use similar logic to movement commands but could be extended
            current_distance = task_status.get('distance_to_target')
            target_reached = task_status.get('target_reached', False)
            system_state = task_status.get('system_state', '')

            # Log task status for debugging
            if current_distance is not None:
                self.logger.debug(f"Follow task status - Distance: {current_distance:.2f}m, Target reached: {target_reached}, State: {system_state}")

            # Check completion conditions for follow commands
            # Follow commands might complete based on different criteria than movement commands
            task_complete = False
            completion_reason = ""

            # For follow_start, we might want to check if the following has been explicitly stopped
            # or if a specific condition is met
            if target_reached:
                task_complete = False
                self.logger.info(f"Currently Reached the target. Staying there but donot terminate following process.")
            # Add other follow-specific completion criteria here as needed

            if task_complete:
                self.logger.info(f"Follow task '{self.active_follow_command}' completed: {completion_reason}")
                self._handle_follow_task_completion()

        except Exception as e:
            self.logger.error(f"Error checking follow task completion: {e}")

    def _handle_movement_task_completion(self):
        """
        Handle the completion of a movement task with automatic state management.
        """
        try:
            # Cancel any active countdown since we're completing via status monitoring
            self._cancel_active_countdown()

            # Stop tracking the movement command
            completed_command = self.active_movement_command
            self._stop_movement_command_tracking()

            # Clear the current task command to prevent re-execution
            self.current_task_command = None

            # Automatically publish stop following command
            stop_success = self._publish_following_command('follow_stop')

            if stop_success:
                self.logger.info(f"Automatically stopped following after completing movement task: '{completed_command}'")

                # RL-based commands stay in RL mode after completion (no standing recovery)
                if completed_command in self.rl_commands_stay_in_rl and self.current_state == TaskState.EXECUTING_TASK:
                    self.logger.info(f"RL-based task '{completed_command}' completed - staying in RL mode, transitioning to idle")
                    # Maintain RL state but transition to idle (don't re-enter RL mode)
                    self._transition_to_state(TaskState.IDLE)
                else:
                    self.logger.info(f"Movement task '{completed_command}' completed - transitioning to idle")
                    self._transition_to_state(TaskState.IDLE)
            else:
                self.logger.warn(f"Failed to automatically stop following after completing movement task: '{completed_command}'")
                # Even if stop following failed, still transition based on RL state
                if completed_command in self.rl_commands_stay_in_rl and self.current_state == TaskState.EXECUTING_TASK:
                    # Maintain RL state but transition to idle (don't re-enter RL mode)
                    self._transition_to_state(TaskState.IDLE)
                else:
                    self._transition_to_state(TaskState.IDLE)

        except Exception as e:
            self.logger.error(f"Error handling movement task completion: {e}")

    def _handle_follow_task_completion(self):
        """
        Handle the completion of a follow task with automatic state management.
        """
        try:
            # Cancel any active countdown since we're completing via status monitoring
            self._cancel_active_countdown()

            # Stop tracking the follow command
            completed_command = self.active_follow_command
            self._stop_follow_command_tracking()

            # Clear the current task command to prevent re-execution
            self.current_task_command = None

            # For follow_start, we might want to automatically stop following
            if completed_command == 'follow_start':
                stop_success = self._publish_following_command('follow_stop')
                if stop_success:
                    self.logger.info(f"Automatically stopped following after completing follow task: '{completed_command}'")
                else:
                    self.logger.warn(f"Failed to automatically stop following after completing follow task: '{completed_command}'")

            # Follow commands stay in RL mode after completion (persistent RL state management)
            if completed_command in self.rl_commands_stay_in_rl and self.current_state == TaskState.EXECUTING_TASK:
                self.logger.info(f"Follow task '{completed_command}' completed - staying in RL mode, transitioning to idle")
                # Maintain RL state but transition to idle (don't re-enter RL mode)
                self._transition_to_state(TaskState.IDLE)
            else:
                self.logger.info(f"Follow task '{completed_command}' completed - transitioning to idle")
                self._transition_to_state(TaskState.IDLE)

        except Exception as e:
            self.logger.error(f"Error handling follow task completion: {e}")

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

    def is_follow_command_active(self):
        """
        Check if a follow command is currently being tracked.

        Returns:
            bool: True if a follow command is active, False otherwise
        """
        return self.active_follow_command is not None

    def get_active_follow_command(self):
        """
        Get the currently active follow command.

        Returns:
            str: Active follow command or None if no command is active
        """
        return self.active_follow_command

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
        self._stop_follow_command_tracking()
        self._cancel_active_countdown()
        self.clear_command_queue()

    def get_current_rl_state(self):
        """
        Get the current RL state of the robot.

        Returns:
            RLState: Current RL state (RL_MODE, STANDING, SITTING, SHAKING_HANDS)
        """
        return self.current_rl_state

    def is_in_rl_mode(self):
        """
        Check if the robot is currently in RL mode.

        Returns:
            bool: True if in RL mode, False if in non-RL mode
        """
        return self.current_rl_state == RLState.RL_MODE

    def is_in_non_rl_mode(self):
        """
        Check if the robot is currently in a non-RL mode (standing, sitting, shaking hands).

        Returns:
            bool: True if in non-RL mode, False if in RL mode
        """
        return self.current_rl_state != RLState.RL_MODE

    def get_rl_state_name(self):
        """
        Get the human-readable name of the current RL state.

        Returns:
            str: Name of the current RL state
        """
        return self.current_rl_state.name

    def set_rl_state(self, rl_state):
        """
        Manually set the RL state. Use with caution.

        Args:
            rl_state (RLState): The RL state to set
        """
        if isinstance(rl_state, RLState):
            old_state = self.current_rl_state
            self.current_rl_state = rl_state
            self.logger.info(f"RL state manually changed: {old_state.name} -> {rl_state.name}")
        else:
            self.logger.error(f"Invalid RL state type: {type(rl_state)}")

    def get_state_summary(self):
        """
        Get a comprehensive summary of the current state.

        Returns:
            dict: Dictionary containing current task state, RL state, and other status info
        """
        return {
            'task_state': self.current_state.name,
            'rl_state': self.current_rl_state.name,
            'current_task': self.current_task_command,
            'is_busy': self.is_busy(),
            'is_in_rl_mode': self.is_in_rl_mode(),
            'active_movement_command': self.active_movement_command,
            'active_follow_command': self.active_follow_command,
            'queued_commands_count': len(self.task_queue),
            'state_duration': self.get_state_duration(),
            'persistent_rl_active': self.current_rl_state == RLState.RL_MODE and self.current_state == TaskState.IDLE
        }

    def should_skip_rl_entry(self, command):
        """
        Determine if RL mode entry should be skipped for a command.

        Args:
            command (str): The command to check

        Returns:
            bool: True if RL entry should be skipped, False otherwise
        """
        return (command in self.rl_mode_required_commands and
                self.current_rl_state == RLState.RL_MODE)

    def log_rl_state_transition(self, command, action_taken):
        """
        Log RL state transitions for debugging and monitoring.

        Args:
            command (str): The command being processed
            action_taken (str): Description of the action taken
        """
        self.logger.info(f"RL State Management - Command: '{command}', "
                        f"Current RL State: {self.current_rl_state.name}, "
                        f"Task State: {self.current_state.name}, "
                        f"Action: {action_taken}")
