#!/usr/bin/env python3
"""
LLM Planning Module for Autonomous Humanoid

Provides cognitive task planning using GPT-4 or similar LLMs,
integrated with the main orchestrator.
"""

import os
import json
from typing import Optional, Dict

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

from dotenv import load_dotenv
load_dotenv()


class LLMPlanningModule:
    """LLM-based task planning module."""

    def __init__(self, parent_node, model: str = "gpt-4"):
        """Initialize LLM planning module.

        Args:
            parent_node: Parent ROS 2 node for logging
            model: OpenAI model to use
        """
        self.node = parent_node
        self.logger = parent_node.get_logger()
        self.model = model

        if OPENAI_AVAILABLE:
            api_key = os.getenv("OPENAI_API_KEY")
            if api_key:
                self.client = OpenAI(api_key=api_key)
                self.logger.info("LLM client initialized")
            else:
                self.client = None
                self.logger.warn("OPENAI_API_KEY not set")
        else:
            self.client = None
            self.logger.warn("OpenAI package not available")

        self.system_prompt = self._build_system_prompt()

    def _build_system_prompt(self) -> str:
        """Build the system prompt for task planning."""
        return """You are a humanoid robot task planner. Convert commands to action sequences.

Available actions:
- navigate: Move to location (params: location_name, x, y)
- detect: Find object (params: object_type, color)
- grasp: Pick up object (params: gripper_force, approach_direction)
- place: Put down object (params: target_location)
- speak: Say message (params: message)
- scan: Survey area (params: scan_type)
- wait: Pause (params: duration_seconds)

Output JSON:
{
    "goal": "<command>",
    "plan": [
        {"step": 1, "type": "<action>", "target": "<target>",
         "parameters": {...}, "description": "<desc>",
         "estimated_duration": <seconds>}
    ],
    "total_estimated_time": <seconds>,
    "confidence": <0.0-1.0>
}

Example:
Command: "Pick up the red cube from the table"
{
    "goal": "Pick up the red cube from the table",
    "plan": [
        {"step": 1, "type": "navigate", "target": "table",
         "parameters": {"location_name": "table"},
         "description": "Move to the table", "estimated_duration": 10.0},
        {"step": 2, "type": "detect", "target": "red cube",
         "parameters": {"object_type": "cube", "color": "red"},
         "description": "Locate the red cube", "estimated_duration": 3.0},
        {"step": 3, "type": "grasp", "target": "red cube",
         "parameters": {"gripper_force": 0.5},
         "description": "Grasp the red cube", "estimated_duration": 5.0},
        {"step": 4, "type": "speak", "target": "user",
         "parameters": {"message": "I have picked up the red cube"},
         "description": "Confirm completion", "estimated_duration": 2.0}
    ],
    "total_estimated_time": 20.0,
    "confidence": 0.95
}"""

    def plan_task(self, command: str) -> Optional[Dict]:
        """Generate a task plan from a natural language command.

        Args:
            command: Natural language command

        Returns:
            Plan dictionary or None on failure
        """
        if not self.client:
            self.logger.error("LLM client not available")
            return None

        try:
            self.logger.info(f"Planning: {command}")

            response = self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self.system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0.0,
                max_tokens=1000
            )

            text = response.choices[0].message.content.strip()

            # Handle markdown code blocks
            if "```json" in text:
                text = text.split("```json")[1].split("```")[0]
            elif "```" in text:
                text = text.split("```")[1].split("```")[0]

            plan = json.loads(text)

            # Validate
            if not self._validate_plan(plan):
                self.logger.error("Invalid plan structure")
                return None

            self.logger.info(f"Generated plan with {len(plan.get('plan', []))} steps")
            return plan

        except json.JSONDecodeError as e:
            self.logger.error(f"JSON parsing error: {e}")
            return None
        except Exception as e:
            self.logger.error(f"Planning error: {e}")
            return None

    def _validate_plan(self, plan: Dict) -> bool:
        """Validate plan structure."""
        if not isinstance(plan, dict):
            return False
        if 'plan' not in plan or not isinstance(plan['plan'], list):
            return False
        if len(plan['plan']) == 0:
            return False
        return True

    def is_available(self) -> bool:
        """Check if planning module is available."""
        return self.client is not None
