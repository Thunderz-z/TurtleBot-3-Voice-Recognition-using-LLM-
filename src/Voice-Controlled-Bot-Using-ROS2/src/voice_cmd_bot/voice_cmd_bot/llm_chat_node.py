#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
from openai import OpenAI
import pyttsx3
import threading

class LLMChatNode(Node):
    def __init__(self):
        super().__init__('llm_chat_node')
        
        # Subscribe to voice commands
        self.subscription = self.create_subscription(
            String, 
            'voice_commands', 
            self.voice_callback, 
            10
        )
        
        # Publisher for LLM responses
        self.response_publisher = self.create_publisher(
            String, 
            'llm_response', 
            10
        )
        
        # Movement commands that should be ignored by LLM
        self.movement_keywords = ['forward', 'backward', 'left', 'right', 'stop']
        
        # Filler words and noise that should be ignored
        self.filler_words = ['huh', 'uh', 'um', 'hmm', 'ah', 'er', 'oh']
        
        # ============================================
        # OPENROUTER API KEY - Set your key here
        # ============================================
        OPENROUTER_API_KEY = ""
        # ============================================
        
        # Initialize OpenRouter client for Dolphin 3.0 Mistral 24B
        self.client = OpenAI(
            base_url="https://openrouter.ai/api/v1",
            api_key=OPENROUTER_API_KEY,
        )
        
        # Model to use (Dolphin 3.0 Mistral 24B - Free)
        self.model_name = "cognitivecomputations/dolphin3.0-mistral-24b:free"
        
        # Store conversation history manually for context
        self.conversation_history = []
        
        # Initialize Text-to-Speech engine
        try:
            self.tts_engine = pyttsx3.init()
            # Set properties for better speech
            self.tts_engine.setProperty('rate', 150)  # Speed of speech
            self.tts_engine.setProperty('volume', 0.9)  # Volume (0.0 to 1.0)
            self.tts_enabled = True
            self.get_logger().info("Text-to-Speech initialized successfully!")
        except Exception as e:
            self.get_logger().warning(f"Could not initialize TTS: {e}. Continuing without speech output.")
            self.tts_enabled = False
        
        self.get_logger().info("LLM Chat Node started. Ready for conversation!")
        self.get_logger().info("Movement commands (forward, backward, left, right, stop) will be ignored.")
        
    def voice_callback(self, msg):
        """
        Callback for voice commands. Filters out movement commands
        and sends conversational text to Gemini LLM.
        """
        command = msg.data.lower().strip()
        
        if not command:
            return
        
        # Filter 1: Ignore very short inputs (likely noise)
        if len(command) < 3:
            self.get_logger().debug(f"Ignoring short input: {command}")
            return
        
        # Filter 2: Check if this is a movement command
        is_movement_command = any(keyword in command for keyword in self.movement_keywords)
        
        if is_movement_command:
            self.get_logger().debug(f"Ignoring movement command: {command}")
            return
        
        # Filter 3: Check if this is just a filler word
        if command in self.filler_words:
            self.get_logger().debug(f"Ignoring filler word: {command}")
            return
        
        # Filter 4: Require at least 2 words for meaningful conversation
        word_count = len(command.split())
        if word_count < 2:
            self.get_logger().debug(f"Ignoring single word input: {command}")
            return
        
        # This is a conversational message, send to LLM
        self.get_logger().info(f"Sending to LLM: {command}")
        
        try:
            # Build messages list for OpenAI-style API
            messages = []
            
            # Add conversation history
            for i in range(0, len(self.conversation_history[-20:]), 2):  # Last 10 exchanges
                if i + 1 < len(self.conversation_history[-20:]):
                    messages.append({
                        "role": "user",
                        "content": self.conversation_history[-20:][i].replace("User: ", "")
                    })
                    messages.append({
                        "role": "assistant",
                        "content": self.conversation_history[-20:][i+1].replace("Assistant: ", "")
                    })
            
            # Add current message
            messages.append({"role": "user", "content": command})
            
            # Send message to Dolphin via OpenRouter
            completion = self.client.chat.completions.create(
                model=self.model_name,
                messages=messages,
            )
            
            llm_response = completion.choices[0].message.content
            
            # Update conversation history
            self.conversation_history.append(f"User: {command}")
            self.conversation_history.append(f"Assistant: {llm_response}")
            
            # Publish the response
            response_msg = String()
            response_msg.data = llm_response
            self.response_publisher.publish(response_msg)
            
            self.get_logger().info(f"LLM Response: {llm_response}")
            
            # Speak the response using TTS (in separate thread to not block)
            if self.tts_enabled:
                tts_thread = threading.Thread(target=self._speak, args=(llm_response,))
                tts_thread.daemon = True
                tts_thread.start()
            
        except Exception as e:
            self.get_logger().error(f"Error communicating with LLM: {str(e)}")
    
    def _speak(self, text):
        """
        Speak the text using TTS in a separate thread.
        """
        try:
            self.tts_engine.say(text)
            self.tts_engine.runAndWait()
        except Exception as e:
            self.get_logger().error(f"TTS Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LLMChatNode()
        rclpy.spin(node)
    except ValueError as e:
        print(f"Error: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
