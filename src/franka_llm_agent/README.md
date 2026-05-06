# Franka LLM Agent

This package provides a natural language interface for the Franka Panda pick-and-place simulation. It allows you to control the robot using free-form text commands.

## Overview
The `llm_agent` node has been upgraded to a **Vision-Language-Action (VLA)** agent. It now combines real-time visual input from the robot's cameras with natural language prompts to decide and execute robot actions.

## Features
- **VLA Integration**: Subscribes to `/wrist_camera/image` and `/overhead_camera/image`.
- **Visual Reasoning**: Processes images to identify target objects based on user prompts.
- **Action Execution**: Directly triggers the `PickAndPlaceTask` action based on visual-linguistic interpretation.
- **API Ready**: Includes helper methods to encode images for external VLA APIs (e.g., GPT-4o, OpenVLA).

## Getting Started

### 1. Build the Package
```bash
colcon build --packages-select franka_llm_agent
source install/setup.bash
```

### 2. Run the Agent
```bash
ros2 run franka_llm_agent llm_agent
```

### 3. Send a Command
You can test the agent by publishing a string to the `/llm_prompt` topic:
```bash
ros2 topic pub --once /llm_prompt std_msgs/msg/String "{data: 'Hey robot, can you pick up the red cube and move it to the bin?'}"
```

## Integrating a Real LLM
Currently, the node uses a rule-based mock for parsing. To use a real LLM:
1. Install your preferred library: `pip install openai` or `pip install langchain`.
2. Update the `mock_llm_call` method in `llm_agent.py` to make an API request.
3. Pass the `system_prompt` defined in the class to the model for context.
