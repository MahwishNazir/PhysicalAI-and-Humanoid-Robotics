#!/usr/bin/env python3
"""
Error Handler for Robot Task Execution

This module handles errors during task execution and implements re-planning
strategies using the LLM to generate alternative approaches when actions fail.

Features:
- Error classification and diagnosis
- LLM-powered re-planning
- Fallback strategy generation
- Error recovery coordination
- Human intervention requests

Usage:
    from error_handler import ErrorHandler

    handler = ErrorHandler(planner)
    recovery_plan = handler.handle_error(failed_action, error_context)
"""

import os
import json
from typing import Dict, Optional, List
from dataclasses import dataclass
from enum import Enum

# OpenAI import with fallback
try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

from dotenv import load_dotenv
load_dotenv()


class ErrorType(Enum):
    """Types of execution errors."""
    NAVIGATION_BLOCKED = "navigation_blocked"
    OBJECT_NOT_FOUND = "object_not_found"
    GRASP_FAILED = "grasp_failed"
    PLACE_FAILED = "place_failed"
    TIMEOUT = "timeout"
    HARDWARE_ERROR = "hardware_error"
    UNKNOWN = "unknown"


class RecoveryStrategy(Enum):
    """Available recovery strategies."""
    RETRY = "retry"          # Retry the same action
    REPLAN = "replan"        # Generate new plan with LLM
    SKIP = "skip"            # Skip this action
    ABORT = "abort"          # Stop execution
    HUMAN_HELP = "human_help"  # Request human intervention


@dataclass
class ErrorContext:
    """Context information about an error."""
    error_type: ErrorType
    action_type: str
    target: str
    message: str
    attempt_number: int
    environment_state: Dict
    original_plan: Dict


@dataclass
class RecoveryPlan:
    """Recovery plan generated for an error."""
    strategy: RecoveryStrategy
    new_actions: List[Dict]
    explanation: str
    confidence: float


class ErrorHandler:
    """Handles errors during task execution with LLM-powered recovery."""

    def __init__(self, model: str = "gpt-4", max_retries: int = 3):
        """Initialize the error handler.

        Args:
            model: OpenAI model for re-planning
            max_retries: Maximum retry attempts before escalating
        """
        self.model = model
        self.max_retries = max_retries

        # Initialize OpenAI client if available
        self.llm_client = None
        if OPENAI_AVAILABLE:
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key:
                self.llm_client = OpenAI(api_key=api_key)

        # Error handling strategies by error type
        self.default_strategies = {
            ErrorType.NAVIGATION_BLOCKED: RecoveryStrategy.REPLAN,
            ErrorType.OBJECT_NOT_FOUND: RecoveryStrategy.REPLAN,
            ErrorType.GRASP_FAILED: RecoveryStrategy.RETRY,
            ErrorType.PLACE_FAILED: RecoveryStrategy.RETRY,
            ErrorType.TIMEOUT: RecoveryStrategy.RETRY,
            ErrorType.HARDWARE_ERROR: RecoveryStrategy.HUMAN_HELP,
            ErrorType.UNKNOWN: RecoveryStrategy.ABORT,
        }

    def classify_error(self, action: Dict, error_message: str) -> ErrorType:
        """Classify an error based on action and message.

        Args:
            action: The failed action
            error_message: Error message from executor

        Returns:
            Classified ErrorType
        """
        message_lower = error_message.lower()
        action_type = action.get('type', '')

        # Navigation errors
        if action_type == 'navigate':
            if 'blocked' in message_lower or 'obstacle' in message_lower:
                return ErrorType.NAVIGATION_BLOCKED
            if 'timeout' in message_lower:
                return ErrorType.TIMEOUT

        # Detection errors
        if action_type == 'detect':
            if 'not found' in message_lower or 'cannot find' in message_lower:
                return ErrorType.OBJECT_NOT_FOUND

        # Manipulation errors
        if action_type == 'grasp':
            if 'failed' in message_lower or 'slip' in message_lower:
                return ErrorType.GRASP_FAILED

        if action_type == 'place':
            if 'failed' in message_lower:
                return ErrorType.PLACE_FAILED

        # Generic errors
        if 'timeout' in message_lower:
            return ErrorType.TIMEOUT
        if 'hardware' in message_lower or 'motor' in message_lower:
            return ErrorType.HARDWARE_ERROR

        return ErrorType.UNKNOWN

    def handle_error(self, context: ErrorContext) -> RecoveryPlan:
        """Handle an error and generate recovery plan.

        Args:
            context: Error context with all relevant information

        Returns:
            RecoveryPlan with strategy and new actions
        """
        # Check retry limit
        if context.attempt_number >= self.max_retries:
            if context.error_type in [ErrorType.GRASP_FAILED, ErrorType.TIMEOUT]:
                # Escalate to re-planning after max retries
                return self._generate_replan(context)

        # Get default strategy
        strategy = self.default_strategies.get(
            context.error_type, RecoveryStrategy.ABORT
        )

        # Handle each strategy
        if strategy == RecoveryStrategy.RETRY:
            return self._generate_retry(context)
        elif strategy == RecoveryStrategy.REPLAN:
            return self._generate_replan(context)
        elif strategy == RecoveryStrategy.HUMAN_HELP:
            return self._request_human_help(context)
        else:
            return self._generate_abort(context)

    def _generate_retry(self, context: ErrorContext) -> RecoveryPlan:
        """Generate a retry strategy."""
        # Get the original action
        failed_action = None
        for action in context.original_plan.get('plan', []):
            if action.get('target') == context.target:
                failed_action = action
                break

        if failed_action is None:
            return self._generate_abort(context)

        return RecoveryPlan(
            strategy=RecoveryStrategy.RETRY,
            new_actions=[failed_action],
            explanation=f"Retrying {context.action_type} (attempt {context.attempt_number + 1}/{self.max_retries})",
            confidence=0.7
        )

    def _generate_replan(self, context: ErrorContext) -> RecoveryPlan:
        """Generate a new plan using LLM."""
        if self.llm_client is None:
            return self._generate_fallback(context)

        # Build re-planning prompt
        prompt = self._build_replan_prompt(context)

        try:
            response = self.llm_client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_replan_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.2,
                max_tokens=800
            )

            response_text = response.choices[0].message.content.strip()

            # Parse JSON response
            if "```json" in response_text:
                response_text = response_text.split("```json")[1].split("```")[0]
            elif "```" in response_text:
                response_text = response_text.split("```")[1].split("```")[0]

            recovery = json.loads(response_text)

            return RecoveryPlan(
                strategy=RecoveryStrategy.REPLAN,
                new_actions=recovery.get('new_actions', []),
                explanation=recovery.get('explanation', 'LLM-generated recovery plan'),
                confidence=recovery.get('confidence', 0.8)
            )

        except Exception as e:
            print(f"Re-planning error: {e}")
            return self._generate_fallback(context)

    def _build_replan_prompt(self, context: ErrorContext) -> str:
        """Build prompt for LLM re-planning."""
        return f"""The robot encountered an error during task execution.

Original Goal: {context.original_plan.get('goal', 'Unknown')}

Failed Action:
- Type: {context.action_type}
- Target: {context.target}
- Error: {context.error_type.value}
- Message: {context.message}
- Attempt: {context.attempt_number}

Environment State:
{json.dumps(context.environment_state, indent=2)}

Original Plan Steps:
{json.dumps(context.original_plan.get('plan', []), indent=2)}

Please generate an alternative approach to achieve the goal, avoiding the failed action or modifying the approach.
"""

    def _get_replan_system_prompt(self) -> str:
        """Get system prompt for re-planning."""
        return """You are a robot task recovery planner. Given a failed action and context, generate an alternative approach.

Output ONLY valid JSON in this format:
{
    "explanation": "<why the original failed and what the new approach does>",
    "new_actions": [
        {"step": 1, "type": "<action>", "target": "<target>", "parameters": {}, "description": "<desc>", "estimated_duration": <sec>}
    ],
    "confidence": <0.0-1.0>
}

Recovery strategies:
- For navigation_blocked: Try alternate route or approach from different angle
- For object_not_found: Expand search area or check alternate locations
- For grasp_failed: Adjust gripper force, approach angle, or reposition
- For place_failed: Adjust placement location or orientation

Keep new plans concise (1-3 actions) and focused on recovery."""

    def _generate_fallback(self, context: ErrorContext) -> RecoveryPlan:
        """Generate a simple fallback plan without LLM."""
        fallback_actions = []

        # Simple fallbacks based on error type
        if context.error_type == ErrorType.NAVIGATION_BLOCKED:
            fallback_actions = [
                {"step": 1, "type": "navigate", "target": "backup_position",
                 "parameters": {"direction": "backward", "distance": 0.5},
                 "description": "Move backward to clear obstacle",
                 "estimated_duration": 3.0}
            ]
        elif context.error_type == ErrorType.OBJECT_NOT_FOUND:
            fallback_actions = [
                {"step": 1, "type": "scan", "target": "area",
                 "parameters": {"scan_type": "full"},
                 "description": "Full area scan for object",
                 "estimated_duration": 10.0}
            ]
        elif context.error_type == ErrorType.GRASP_FAILED:
            fallback_actions = [
                {"step": 1, "type": "grasp", "target": context.target,
                 "parameters": {"gripper_force": 0.7, "approach_direction": "side"},
                 "description": "Retry grasp with increased force",
                 "estimated_duration": 5.0}
            ]

        return RecoveryPlan(
            strategy=RecoveryStrategy.REPLAN,
            new_actions=fallback_actions,
            explanation=f"Fallback recovery for {context.error_type.value}",
            confidence=0.5
        )

    def _request_human_help(self, context: ErrorContext) -> RecoveryPlan:
        """Request human intervention."""
        return RecoveryPlan(
            strategy=RecoveryStrategy.HUMAN_HELP,
            new_actions=[
                {"step": 1, "type": "speak", "target": "user",
                 "parameters": {"message": f"I need help. {context.message}"},
                 "description": "Request human assistance",
                 "estimated_duration": 3.0},
                {"step": 2, "type": "wait", "target": "user",
                 "parameters": {"duration_seconds": 30},
                 "description": "Wait for human intervention",
                 "estimated_duration": 30.0}
            ],
            explanation=f"Hardware or critical error: {context.message}. Human intervention required.",
            confidence=1.0
        )

    def _generate_abort(self, context: ErrorContext) -> RecoveryPlan:
        """Generate abort plan."""
        return RecoveryPlan(
            strategy=RecoveryStrategy.ABORT,
            new_actions=[
                {"step": 1, "type": "speak", "target": "user",
                 "parameters": {"message": f"Task aborted: {context.message}"},
                 "description": "Announce task abort",
                 "estimated_duration": 2.0}
            ],
            explanation=f"Unrecoverable error: {context.message}",
            confidence=1.0
        )


def main():
    """Demo the error handler."""
    print("\n" + "=" * 60)
    print("Error Handler Demo")
    print("=" * 60 + "\n")

    handler = ErrorHandler()

    # Create test error context
    original_plan = {
        "goal": "Pick up the red cup",
        "plan": [
            {"step": 1, "type": "navigate", "target": "table",
             "parameters": {"location_name": "table"},
             "description": "Move to table"},
            {"step": 2, "type": "detect", "target": "red cup",
             "parameters": {"object_type": "cup", "color": "red"},
             "description": "Find red cup"},
            {"step": 3, "type": "grasp", "target": "red cup",
             "parameters": {"gripper_force": 0.5},
             "description": "Grasp cup"}
        ]
    }

    # Test error scenarios
    test_errors = [
        ErrorContext(
            error_type=ErrorType.OBJECT_NOT_FOUND,
            action_type="detect",
            target="red cup",
            message="Cannot find red cup on table",
            attempt_number=1,
            environment_state={"robot_position": [1.0, 0.0, 0.0], "objects_visible": []},
            original_plan=original_plan
        ),
        ErrorContext(
            error_type=ErrorType.GRASP_FAILED,
            action_type="grasp",
            target="red cup",
            message="Gripper slipped, object dropped",
            attempt_number=2,
            environment_state={"robot_position": [1.0, 0.5, 0.0], "object_position": [1.0, 0.5, 0.7]},
            original_plan=original_plan
        ),
        ErrorContext(
            error_type=ErrorType.HARDWARE_ERROR,
            action_type="grasp",
            target="red cup",
            message="Gripper motor fault detected",
            attempt_number=1,
            environment_state={},
            original_plan=original_plan
        ),
    ]

    for i, error in enumerate(test_errors):
        print(f"\n--- Error Scenario {i+1} ---")
        print(f"Error Type: {error.error_type.value}")
        print(f"Message: {error.message}")

        recovery = handler.handle_error(error)

        print(f"\nRecovery Strategy: {recovery.strategy.value}")
        print(f"Explanation: {recovery.explanation}")
        print(f"Confidence: {recovery.confidence}")
        print(f"New Actions: {len(recovery.new_actions)}")
        for action in recovery.new_actions:
            print(f"  - {action.get('description', action.get('type'))}")

    print("\nDemo complete.")


if __name__ == "__main__":
    main()
