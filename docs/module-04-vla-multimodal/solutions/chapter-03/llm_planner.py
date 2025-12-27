#!/usr/bin/env python3
"""
LLM Task Planner for Robotics

This module implements cognitive planning using Large Language Models to decompose
high-level natural language commands into executable robot action sequences.

Features:
- Few-shot prompted task decomposition
- JSON-structured action output
- Multi-step plan validation
- Support for various action types (navigate, detect, grasp, place)

Usage:
    from llm_planner import LLMPlanner

    planner = LLMPlanner()
    plan = planner.plan_task("Pick up the red cup from the table")
    for action in plan:
        print(f"Action: {action['type']}, Target: {action['target']}")
"""

import os
import json
from typing import List, Dict, Optional
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


class ActionType(Enum):
    """Supported robot action types."""
    NAVIGATE = "navigate"
    DETECT = "detect"
    GRASP = "grasp"
    PLACE = "place"
    SPEAK = "speak"
    WAIT = "wait"
    SCAN = "scan"


@dataclass
class RobotAction:
    """Represents a single robot action in a task plan."""
    action_type: ActionType
    target: str
    parameters: Dict
    description: str
    estimated_duration: float  # seconds

    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return {
            'type': self.action_type.value,
            'target': self.target,
            'parameters': self.parameters,
            'description': self.description,
            'estimated_duration': self.estimated_duration
        }

    @classmethod
    def from_dict(cls, data: Dict) -> 'RobotAction':
        """Create from dictionary."""
        return cls(
            action_type=ActionType(data['type']),
            target=data.get('target', ''),
            parameters=data.get('parameters', {}),
            description=data.get('description', ''),
            estimated_duration=data.get('estimated_duration', 5.0)
        )


class LLMPlanner:
    """LLM-based task planner for robot action sequences."""

    def __init__(self, model: str = "gpt-4", temperature: float = 0.0):
        """Initialize the LLM planner.

        Args:
            model: OpenAI model to use
            temperature: LLM temperature (0.0 for deterministic)
        """
        if not OPENAI_AVAILABLE:
            raise ImportError("OpenAI package not installed. Install with: pip install openai")

        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable not set")

        self.client = OpenAI(api_key=api_key)
        self.model = model
        self.temperature = temperature

        # System prompt with few-shot examples
        self.system_prompt = self._build_system_prompt()

    def _build_system_prompt(self) -> str:
        """Build the system prompt with few-shot examples."""
        return """You are a robot task planner. Given a high-level command, break it down into a sequence of executable actions.

Available action types:
- navigate: Move to a location (parameters: x, y, z coordinates or location_name)
- detect: Find and locate an object (parameters: object_type, color, size)
- grasp: Pick up an object (parameters: gripper_force, approach_direction)
- place: Put down an object (parameters: target_location, orientation)
- speak: Say something (parameters: message)
- wait: Pause execution (parameters: duration_seconds)
- scan: Survey the environment (parameters: scan_type)

Output ONLY valid JSON in this format:
{
    "goal": "<original command>",
    "plan": [
        {
            "step": 1,
            "type": "<action_type>",
            "target": "<target object/location>",
            "parameters": {<action parameters>},
            "description": "<human-readable description>",
            "estimated_duration": <seconds>
        }
    ],
    "total_estimated_time": <total seconds>,
    "confidence": <0.0-1.0>
}

Examples:

Command: "Pick up the red cup from the table"
{
    "goal": "Pick up the red cup from the table",
    "plan": [
        {"step": 1, "type": "navigate", "target": "table", "parameters": {"location_name": "table"}, "description": "Move to the table", "estimated_duration": 10.0},
        {"step": 2, "type": "detect", "target": "red cup", "parameters": {"object_type": "cup", "color": "red"}, "description": "Locate the red cup on the table", "estimated_duration": 3.0},
        {"step": 3, "type": "grasp", "target": "red cup", "parameters": {"gripper_force": 0.5, "approach_direction": "top"}, "description": "Grasp the red cup", "estimated_duration": 5.0}
    ],
    "total_estimated_time": 18.0,
    "confidence": 0.95
}

Command: "Clean the room"
{
    "goal": "Clean the room",
    "plan": [
        {"step": 1, "type": "scan", "target": "room", "parameters": {"scan_type": "full"}, "description": "Survey the room for objects out of place", "estimated_duration": 15.0},
        {"step": 2, "type": "navigate", "target": "first_object", "parameters": {"location_name": "detected_object_1"}, "description": "Move to first detected object", "estimated_duration": 8.0},
        {"step": 3, "type": "detect", "target": "object", "parameters": {"object_type": "any"}, "description": "Identify the object", "estimated_duration": 2.0},
        {"step": 4, "type": "grasp", "target": "object", "parameters": {"gripper_force": 0.5}, "description": "Pick up the object", "estimated_duration": 5.0},
        {"step": 5, "type": "navigate", "target": "storage", "parameters": {"location_name": "storage_bin"}, "description": "Move to storage location", "estimated_duration": 10.0},
        {"step": 6, "type": "place", "target": "storage_bin", "parameters": {"target_location": "storage_bin"}, "description": "Place object in storage", "estimated_duration": 3.0},
        {"step": 7, "type": "speak", "target": "user", "parameters": {"message": "Room cleaning complete"}, "description": "Announce completion", "estimated_duration": 2.0}
    ],
    "total_estimated_time": 45.0,
    "confidence": 0.85
}

Command: "Go to the kitchen"
{
    "goal": "Go to the kitchen",
    "plan": [
        {"step": 1, "type": "navigate", "target": "kitchen", "parameters": {"location_name": "kitchen"}, "description": "Navigate to the kitchen", "estimated_duration": 15.0}
    ],
    "total_estimated_time": 15.0,
    "confidence": 0.98
}
"""

    def plan_task(self, command: str) -> Optional[Dict]:
        """Generate a task plan from a natural language command.

        Args:
            command: Natural language command (e.g., "Pick up the red cup")

        Returns:
            Dict containing the plan, or None on failure
        """
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=self.temperature,
                max_tokens=1000
            )

            # Parse response
            response_text = response.choices[0].message.content.strip()

            # Extract JSON (handle potential markdown code blocks)
            if "```json" in response_text:
                response_text = response_text.split("```json")[1].split("```")[0]
            elif "```" in response_text:
                response_text = response_text.split("```")[1].split("```")[0]

            plan = json.loads(response_text)

            # Validate plan structure
            if not self._validate_plan(plan):
                print(f"Invalid plan structure: {plan}")
                return None

            return plan

        except json.JSONDecodeError as e:
            print(f"JSON parsing error: {e}")
            return None
        except Exception as e:
            print(f"Planning error: {e}")
            return None

    def _validate_plan(self, plan: Dict) -> bool:
        """Validate the plan structure.

        Args:
            plan: Plan dictionary from LLM

        Returns:
            True if valid, False otherwise
        """
        required_keys = ['goal', 'plan', 'total_estimated_time', 'confidence']
        if not all(key in plan for key in required_keys):
            return False

        if not isinstance(plan['plan'], list) or len(plan['plan']) == 0:
            return False

        valid_types = {at.value for at in ActionType}
        for action in plan['plan']:
            if 'type' not in action or action['type'] not in valid_types:
                return False
            if 'step' not in action:
                return False

        return True

    def get_action_sequence(self, command: str) -> List[RobotAction]:
        """Get a list of RobotAction objects from a command.

        Args:
            command: Natural language command

        Returns:
            List of RobotAction objects
        """
        plan = self.plan_task(command)
        if plan is None:
            return []

        actions = []
        for action_dict in plan['plan']:
            try:
                action = RobotAction.from_dict(action_dict)
                actions.append(action)
            except (ValueError, KeyError) as e:
                print(f"Error parsing action: {e}")
                continue

        return actions

    def explain_plan(self, plan: Dict) -> str:
        """Generate a human-readable explanation of the plan.

        Args:
            plan: Plan dictionary

        Returns:
            Human-readable explanation string
        """
        if plan is None:
            return "No plan available"

        explanation = [f"Goal: {plan['goal']}\n"]
        explanation.append(f"Estimated time: {plan['total_estimated_time']}s")
        explanation.append(f"Confidence: {plan['confidence']*100:.0f}%\n")
        explanation.append("Steps:")

        for action in plan['plan']:
            step = action['step']
            desc = action.get('description', action['type'])
            duration = action.get('estimated_duration', '?')
            explanation.append(f"  {step}. {desc} ({duration}s)")

        return "\n".join(explanation)


def main():
    """Demo the LLM planner."""
    print("\n" + "=" * 60)
    print("LLM Task Planner Demo")
    print("=" * 60 + "\n")

    if not OPENAI_AVAILABLE:
        print("ERROR: OpenAI package not installed")
        print("Install with: pip install openai")
        return

    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        print("ERROR: OPENAI_API_KEY not set")
        print("Set with: export OPENAI_API_KEY='your-key-here'")
        return

    try:
        planner = LLMPlanner()

        # Test commands
        test_commands = [
            "Pick up the red cup from the table",
            "Go to the kitchen",
            "Clean the room",
            "Find and bring me my keys",
        ]

        for command in test_commands:
            print(f"\nCommand: \"{command}\"")
            print("-" * 50)

            plan = planner.plan_task(command)
            if plan:
                print(planner.explain_plan(plan))
            else:
                print("Failed to generate plan")

            print()

    except Exception as e:
        print(f"Error: {e}")


if __name__ == "__main__":
    main()
