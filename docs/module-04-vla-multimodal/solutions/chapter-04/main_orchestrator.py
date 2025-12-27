#!/usr/bin/env python3
"""
Main Orchestrator for Autonomous Humanoid Robot

This module coordinates all components (voice, planning, navigation, vision,
manipulation) into a unified autonomous system. It implements a state machine
for task execution and handles the complete pipeline from voice command to
physical action.

Features:
- Component coordination and lifecycle management
- State machine for task phases
- Error handling and recovery coordination
- System health monitoring
- Graceful shutdown

Usage:
    ros2 run vla_capstone main_orchestrator

    # Or run directly
    python3 main_orchestrator.py
"""

import threading
from typing import Optional, Dict, List, Callable
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import Twist

# Import local modules
from voice_interface import VoiceInterface
from llm_planning import LLMPlanningModule
from navigation_controller import NavigationController
from vision_module import VisionModule
from manipulation_controller import ManipulationController


class SystemState(Enum):
    """System state machine states."""
    IDLE = "idle"
    LISTENING = "listening"
    PLANNING = "planning"
    NAVIGATING = "navigating"
    DETECTING = "detecting"
    MANIPULATING = "manipulating"
    SPEAKING = "speaking"
    ERROR = "error"
    SHUTDOWN = "shutdown"


@dataclass
class TaskContext:
    """Context for current task execution."""
    command: str
    plan: Optional[Dict] = None
    current_step: int = 0
    detected_objects: List[Dict] = None
    error_message: Optional[str] = None

    def __post_init__(self):
        if self.detected_objects is None:
            self.detected_objects = []


class MainOrchestrator(Node):
    """Main orchestrator coordinating all autonomous system components."""

    def __init__(self):
        super().__init__('main_orchestrator')

        # Callback groups for concurrent operations
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.service_group = ReentrantCallbackGroup()

        # System state
        self.state = SystemState.IDLE
        self.current_task: Optional[TaskContext] = None
        self.is_running = True

        # State transition handlers
        self.state_handlers = {
            SystemState.IDLE: self._handle_idle,
            SystemState.LISTENING: self._handle_listening,
            SystemState.PLANNING: self._handle_planning,
            SystemState.NAVIGATING: self._handle_navigating,
            SystemState.DETECTING: self._handle_detecting,
            SystemState.MANIPULATING: self._handle_manipulating,
            SystemState.SPEAKING: self._handle_speaking,
            SystemState.ERROR: self._handle_error,
        }

        # Initialize modules
        self._init_modules()

        # Publishers
        self.status_pub = self.create_publisher(String, '/system_status', 10)
        self.speech_pub = self.create_publisher(String, '/speech_output', 10)

        # Subscribers
        self.voice_sub = self.create_subscription(
            String, '/voice_command', self._voice_callback, 10)

        # Main loop timer
        self.main_timer = self.create_timer(
            0.1, self._main_loop, callback_group=self.timer_group)

        self._publish_status("System initialized and ready")
        self.get_logger().info("Main orchestrator initialized")

    def _init_modules(self):
        """Initialize all system modules."""
        self.get_logger().info("Initializing system modules...")

        # Voice interface (Whisper)
        try:
            self.voice = VoiceInterface(self)
            self.get_logger().info("Voice interface initialized")
        except Exception as e:
            self.get_logger().warn(f"Voice interface unavailable: {e}")
            self.voice = None

        # LLM Planning
        try:
            self.planner = LLMPlanningModule(self)
            self.get_logger().info("LLM planner initialized")
        except Exception as e:
            self.get_logger().warn(f"LLM planner unavailable: {e}")
            self.planner = None

        # Navigation
        try:
            self.navigation = NavigationController(self)
            self.get_logger().info("Navigation controller initialized")
        except Exception as e:
            self.get_logger().warn(f"Navigation unavailable: {e}")
            self.navigation = None

        # Vision
        try:
            self.vision = VisionModule(self)
            self.get_logger().info("Vision module initialized")
        except Exception as e:
            self.get_logger().warn(f"Vision module unavailable: {e}")
            self.vision = None

        # Manipulation
        try:
            self.manipulation = ManipulationController(self)
            self.get_logger().info("Manipulation controller initialized")
        except Exception as e:
            self.get_logger().warn(f"Manipulation unavailable: {e}")
            self.manipulation = None

    def _publish_status(self, message: str):
        """Publish system status message."""
        msg = String()
        msg.data = f"[{self.state.value}] {message}"
        self.status_pub.publish(msg)
        self.get_logger().info(msg.data)

    def _speak(self, message: str):
        """Publish speech output."""
        msg = String()
        msg.data = message
        self.speech_pub.publish(msg)

    def _voice_callback(self, msg: String):
        """Handle incoming voice commands."""
        if self.state == SystemState.IDLE:
            command = msg.data.strip()
            if command:
                self.get_logger().info(f"Received command: {command}")
                self.current_task = TaskContext(command=command)
                self._transition_to(SystemState.PLANNING)

    def _transition_to(self, new_state: SystemState):
        """Transition to a new state."""
        old_state = self.state
        self.state = new_state
        self._publish_status(f"Transitioned from {old_state.value} to {new_state.value}")

    def _main_loop(self):
        """Main state machine loop."""
        if not self.is_running:
            return

        handler = self.state_handlers.get(self.state)
        if handler:
            try:
                handler()
            except Exception as e:
                self.get_logger().error(f"Error in state {self.state}: {e}")
                if self.current_task:
                    self.current_task.error_message = str(e)
                self._transition_to(SystemState.ERROR)

    def _handle_idle(self):
        """Handle idle state - waiting for commands."""
        pass  # Just wait for voice callback

    def _handle_listening(self):
        """Handle listening state - capturing voice input."""
        if self.voice:
            command = self.voice.capture_command()
            if command:
                self.current_task = TaskContext(command=command)
                self._transition_to(SystemState.PLANNING)
        else:
            # No voice module, stay idle
            self._transition_to(SystemState.IDLE)

    def _handle_planning(self):
        """Handle planning state - LLM task decomposition."""
        if not self.current_task:
            self._transition_to(SystemState.IDLE)
            return

        if not self.planner:
            self._speak("Planning module not available")
            self._transition_to(SystemState.ERROR)
            return

        self._speak(f"Planning task: {self.current_task.command}")

        plan = self.planner.plan_task(self.current_task.command)
        if plan:
            self.current_task.plan = plan
            self.current_task.current_step = 0
            self._publish_status(f"Generated plan with {len(plan.get('plan', []))} steps")
            self._execute_next_step()
        else:
            self.current_task.error_message = "Failed to generate plan"
            self._transition_to(SystemState.ERROR)

    def _execute_next_step(self):
        """Execute the next step in the current plan."""
        if not self.current_task or not self.current_task.plan:
            self._transition_to(SystemState.IDLE)
            return

        actions = self.current_task.plan.get('plan', [])
        if self.current_task.current_step >= len(actions):
            # All steps completed
            self._speak("Task completed successfully")
            self._transition_to(SystemState.IDLE)
            self.current_task = None
            return

        action = actions[self.current_task.current_step]
        action_type = action.get('type', 'unknown')

        self._publish_status(f"Step {self.current_task.current_step + 1}: {action.get('description', action_type)}")

        # Transition to appropriate state based on action type
        state_map = {
            'navigate': SystemState.NAVIGATING,
            'detect': SystemState.DETECTING,
            'grasp': SystemState.MANIPULATING,
            'place': SystemState.MANIPULATING,
            'speak': SystemState.SPEAKING,
            'scan': SystemState.DETECTING,
        }

        next_state = state_map.get(action_type, SystemState.ERROR)
        self._transition_to(next_state)

    def _handle_navigating(self):
        """Handle navigation state."""
        if not self.navigation:
            self._complete_step_fallback("Navigation not available")
            return

        action = self._get_current_action()
        if not action:
            return

        target = action.get('target', 'unknown')
        params = action.get('parameters', {})

        success = self.navigation.navigate_to(
            target,
            params.get('x', 0),
            params.get('y', 0)
        )

        if success:
            self._complete_current_step()
        else:
            self.current_task.error_message = f"Navigation to {target} failed"
            self._transition_to(SystemState.ERROR)

    def _handle_detecting(self):
        """Handle detection state."""
        if not self.vision:
            self._complete_step_fallback("Vision not available")
            return

        action = self._get_current_action()
        if not action:
            return

        target = action.get('target', 'object')
        params = action.get('parameters', {})

        result = self.vision.detect_object(
            params.get('object_type', target),
            params.get('color')
        )

        if result:
            self.current_task.detected_objects.append(result)
            self._complete_current_step()
        else:
            self.current_task.error_message = f"Could not detect {target}"
            self._transition_to(SystemState.ERROR)

    def _handle_manipulating(self):
        """Handle manipulation state."""
        if not self.manipulation:
            self._complete_step_fallback("Manipulation not available")
            return

        action = self._get_current_action()
        if not action:
            return

        action_type = action.get('type')
        target = action.get('target', 'object')
        params = action.get('parameters', {})

        if action_type == 'grasp':
            success = self.manipulation.grasp(target, params.get('gripper_force', 0.5))
        elif action_type == 'place':
            success = self.manipulation.place(params.get('target_location', target))
        else:
            success = False

        if success:
            self._complete_current_step()
        else:
            self.current_task.error_message = f"Manipulation failed: {action_type}"
            self._transition_to(SystemState.ERROR)

    def _handle_speaking(self):
        """Handle speaking state."""
        action = self._get_current_action()
        if action:
            message = action.get('parameters', {}).get('message', 'Hello')
            self._speak(message)
        self._complete_current_step()

    def _handle_error(self):
        """Handle error state - attempt recovery or abort."""
        error_msg = "Unknown error"
        if self.current_task and self.current_task.error_message:
            error_msg = self.current_task.error_message

        self._speak(f"Error: {error_msg}")
        self._publish_status(f"Error: {error_msg}")

        # Reset and go back to idle
        self.current_task = None
        self._transition_to(SystemState.IDLE)

    def _get_current_action(self) -> Optional[Dict]:
        """Get the current action from the plan."""
        if not self.current_task or not self.current_task.plan:
            return None
        actions = self.current_task.plan.get('plan', [])
        if self.current_task.current_step < len(actions):
            return actions[self.current_task.current_step]
        return None

    def _complete_current_step(self):
        """Mark current step complete and move to next."""
        if self.current_task:
            self.current_task.current_step += 1
            self._execute_next_step()

    def _complete_step_fallback(self, message: str):
        """Complete step with fallback behavior when module unavailable."""
        self._publish_status(f"Fallback: {message}")
        self._complete_current_step()

    def shutdown(self):
        """Graceful shutdown."""
        self.is_running = False
        self.state = SystemState.SHUTDOWN
        self._publish_status("Shutting down")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    print("\n" + "=" * 60)
    print("Autonomous Humanoid Robot System")
    print("=" * 60)
    print("\nInitializing system components...")

    try:
        orchestrator = MainOrchestrator()

        # Use multi-threaded executor for concurrent callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(orchestrator)

        print("\nSystem ready. Listening for voice commands...")
        print("Or publish to /voice_command topic")
        print("Press Ctrl+C to shutdown\n")

        executor.spin()

    except KeyboardInterrupt:
        print("\nShutdown requested")
    except Exception as e:
        print(f"\nFatal error: {e}")
    finally:
        if 'orchestrator' in locals():
            orchestrator.shutdown()
            orchestrator.destroy_node()
        rclpy.shutdown()
        print("System shutdown complete.")


if __name__ == "__main__":
    main()
