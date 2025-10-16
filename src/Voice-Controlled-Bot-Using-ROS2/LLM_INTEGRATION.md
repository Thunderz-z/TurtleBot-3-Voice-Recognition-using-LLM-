# LLM Chat Integration for Voice-Controlled Bot

This enhancement adds conversational AI capabilities to your ROS2 voice-controlled bot using Google's Gemini API, without interfering with existing movement commands.

## What's New

A new independent ROS2 node (`llm_chat_node`) that:
- Listens to the same `/voice_commands` topic
- Filters out movement commands (forward, backward, left, right, stop)
- Sends conversational text to Google Gemini LLM
- Publishes LLM responses to `/llm_response` topic

## Architecture

```
Microphone → voice_node → /voice_commands topic
                                 ↓
                    ┌────────────┴────────────┐
                    ↓                         ↓
            command_mapper              llm_chat_node
            (movement cmds)           (conversation)
                    ↓                         ↓
              /cmd_vel topic          /llm_response topic
                    ↓                         ↓
                  Robot                   Console
```

## Setup Instructions

### 1. Get Gemini API Key
1. Go to [Google AI Studio](https://makersuite.google.com/app/apikey)
2. Create a new API key
3. Copy your API key

### 2. Set Environment Variable

**Option A: Temporary (current terminal only)**
```bash
export GEMINI_API_KEY='your_gemini_api_key_here'
```

**Option B: Permanent (recommended)**
Add to your `~/.bashrc`:
```bash
echo "export GEMINI_API_KEY='your_gemini_api_key_here'" >> ~/.bashrc
source ~/.bashrc
```

### 3. Install Dependencies

Navigate to your workspace and rebuild:
```bash
cd ~/voice_ws
colcon build --packages-select voice_cmd_bot
source install/setup.bash
```

The build will automatically install the `google-generativeai` package.

## How to Use

### Running the Complete System

**Terminal 1** - Launch Gazebo simulation:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

**Terminal 2** - Start voice recognition:
```bash
export GEMINI_API_KEY='your_api_key'  # if not set permanently
source ~/voice_ws/install/setup.bash
ros2 run voice_cmd_bot voice_node
```

**Terminal 3** - Start command mapper (robot movement):
```bash
source ~/voice_ws/install/setup.bash
ros2 run voice_cmd_bot command_mapper
```

**Terminal 4** - Start LLM chat node (NEW!):
```bash
export GEMINI_API_KEY='your_api_key'  # if not set permanently
source ~/voice_ws/install/setup.bash
ros2 run voice_cmd_bot llm_chat_node
```

### Command Types

**Movement Commands** (handled by `command_mapper`):
- "forward" - Move robot forward
- "backward" - Move robot backward
- "left" - Turn robot left
- "right" - Turn robot right
- "stop" - Stop the robot

**Conversational Queries** (handled by `llm_chat_node`):
- "What is your name?"
- "Tell me a joke"
- "Explain quantum physics"
- Any other non-movement text

### Monitoring LLM Responses

To see LLM responses in real-time:
```bash
ros2 topic echo /llm_response
```

## Features

✅ **Independent Operation**: LLM chat runs independently without affecting robot movement
✅ **Smart Filtering**: Movement commands automatically bypass LLM processing
✅ **Conversational Context**: Gemini maintains conversation history
✅ **Non-Intrusive**: Existing nodes remain completely unchanged
✅ **Optional**: You can run the system with or without the LLM node

