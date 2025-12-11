---
title: Voice-to-Action
sidebar_position: 2
description: Using OpenAI Whisper for voice commands in humanoid robotics
---

# Voice-to-Action: Using OpenAI Whisper for Voice Commands

## Overview

Voice-to-Action systems enable humanoid robots to receive and interpret spoken commands, bridging the gap between natural human communication and robotic action execution. This module focuses on using OpenAI Whisper for voice recognition and command interpretation in humanoid robotics applications.

## Key Features

### Speech Recognition
- High-accuracy speech-to-text conversion
- Multi-language support
- Noise cancellation and filtering
- Real-time processing capabilities

### Natural Language Understanding
- Intent recognition from spoken commands
- Entity extraction for specific objects or locations
- Context-aware command interpretation
- Ambiguity resolution

### Robot Action Translation
- Mapping natural language to robot actions
- Task decomposition from high-level commands
- Error recovery and clarification requests
- Feedback and confirmation mechanisms

## Architecture Overview

### Voice Processing Pipeline
```
Voice Input → Audio Preprocessing → Speech Recognition → NLU → Action Planning → Robot Execution
```

### Components Integration
- **Audio Input System**: Microphone array with noise cancellation
- **Speech Recognition**: OpenAI Whisper for transcription
- **Natural Language Understanding**: Intent and entity recognition
- **Action Mapping**: Natural language to ROS 2 actions
- **Execution System**: Robot control and feedback

## OpenAI Whisper Integration

### Setup and Configuration
```python
import whisper
import torch
import rospy
from std_msgs.msg import String

class VoiceToActionNode:
    def __init__(self):
        rospy.init_node('voice_to_action')

        # Load Whisper model
        self.model = whisper.load_model("medium.en")  # or multilingual model

        # Audio input subscriber
        self.audio_sub = rospy.Subscriber('/audio_input', AudioData, self.audio_callback)

        # Command publisher
        self.command_pub = rospy.Publisher('/robot_commands', String, queue_size=10)

        # Status publisher
        self.status_pub = rospy.Publisher('/voice_status', String, queue_size=10)

    def audio_callback(self, audio_data):
        """Process incoming audio data"""
        try:
            # Convert audio data to numpy array
            audio_array = self.audio_to_numpy(audio_data)

            # Transcribe using Whisper
            result = self.model.transcribe(audio_array)
            text = result['text']

            # Process the recognized text
            self.process_command(text)

        except Exception as e:
            rospy.logerr(f"Error processing audio: {e}")
            self.status_pub.publish(f"Recognition error: {str(e)}")
```

### Whisper Model Selection
- **tiny**: Fastest, lowest accuracy (good for real-time applications)
- **base**: Good balance of speed and accuracy
- **small**: Better accuracy, slower processing
- **medium**: High accuracy, good for complex commands
- **large**: Highest accuracy, best for detailed instructions

### Real-time Processing
```python
def process_audio_stream(self, audio_chunk):
    """Process audio in real-time chunks"""
    # Convert chunk to appropriate format
    audio_np = self.format_audio_chunk(audio_chunk)

    # Transcribe with confidence threshold
    result = self.model.transcribe(
        audio_np,
        language='en',
        temperature=0.0,  # More deterministic output
        best_of=1
    )

    # Check confidence score
    if result['text'] and len(result['text'].strip()) > 0:
        confidence = result.get('avg_logprob', -1.0)
        if confidence > -0.5:  # Threshold for acceptable confidence
            self.process_command(result['text'])
```

## Natural Language Understanding

### Intent Recognition
```python
class IntentRecognizer:
    def __init__(self):
        # Define command patterns
        self.command_patterns = {
            'navigation': [
                r'go to (?P<location>.+)',
                r'move to (?P<location>.+)',
                r'walk to (?P<location>.+)',
                r'navigate to (?P<location>.+)'
            ],
            'manipulation': [
                r'pick up (?P<object>.+)',
                r'grab (?P<object>.+)',
                r'lift (?P<object>.+)',
                r'put (?P<object>.+) on (?P<destination>.+)'
            ],
            'social_interaction': [
                r'say hello',
                r'wave',
                r'nod',
                r'greet (?P<entity>.+)'
            ]
        }

    def recognize_intent(self, text):
        """Recognize intent and extract entities from text"""
        text_lower = text.lower().strip()

        for intent, patterns in self.command_patterns.items():
            for pattern in patterns:
                match = re.search(pattern, text_lower)
                if match:
                    return {
                        'intent': intent,
                        'entities': match.groupdict(),
                        'confidence': 0.9  # High confidence for pattern matches
                    }

        # If no pattern matches, use more sophisticated NLP
        return self.fallback_nlu(text)
```

### Entity Resolution
- **Locations**: Room names, object positions, coordinates
- **Objects**: Named entities, categories, attributes
- **Actions**: Specific robot capabilities
- **Modifiers**: Speed, precision, repetition

## Action Mapping System

### Natural Language to ROS 2 Actions
```python
class ActionMapper:
    def __init__(self):
        # ROS 2 publishers and services
        self.nav_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.manipulation_client = actionlib.SimpleActionClient('/manipulation_server', ManipulationAction)

    def map_command_to_action(self, parsed_command):
        """Map parsed command to ROS 2 action"""
        intent = parsed_command['intent']
        entities = parsed_command['entities']

        if intent == 'navigation':
            return self.handle_navigation(entities)
        elif intent == 'manipulation':
            return self.handle_manipulation(entities)
        elif intent == 'social_interaction':
            return self.handle_social_interaction(entities)
        else:
            return self.handle_unknown_command()

    def handle_navigation(self, entities):
        """Handle navigation commands"""
        location = entities.get('location', '').lower()

        # Map location name to coordinates
        location_map = {
            'kitchen': (1.0, 2.0, 0.0),
            'living room': (5.0, 3.0, 1.57),
            'bedroom': (2.0, 5.0, 3.14),
        }

        if location in location_map:
            x, y, theta = location_map[location]
            self.send_navigation_goal(x, y, theta)
        else:
            # Request clarification
            self.request_clarification(f"Unknown location: {location}. Please specify a known location.")
```

### Task Decomposition
```python
def decompose_task(self, high_level_command):
    """Decompose high-level commands into sequences of actions"""
    # Example: "Clean the kitchen"
    if "clean the kitchen" in high_level_command.lower():
        return [
            {"action": "navigate", "params": {"location": "kitchen"}},
            {"action": "look_around", "params": {}},
            {"action": "detect_objects", "params": {"category": "trash"}},
            {"action": "pickup_object", "params": {"object": "cup"}},
            {"action": "navigate", "params": {"location": "sink"}},
            {"action": "place_object", "params": {"location": "sink"}},
            {"action": "return", "params": {"location": "starting_position"}}
        ]

    # Example: "Bring me coffee from the kitchen"
    elif "bring me coffee" in high_level_command.lower():
        return [
            {"action": "navigate", "params": {"location": "kitchen"}},
            {"action": "locate", "params": {"object": "coffee"}},
            {"action": "grasp", "params": {"object": "coffee", "hand": "right"}},
            {"action": "navigate", "params": {"location": "user_location"}},
            {"action": "present", "params": {"object": "coffee", "hand": "right"}}
        ]
```

## Audio Processing and Noise Reduction

### Microphone Array Processing
```python
import numpy as np
from scipy import signal

class AudioProcessor:
    def __init__(self):
        # Beamforming parameters
        self.mic_positions = [(0, 0), (0.1, 0), (0, 0.1), (-0.1, 0)]  # Example quad mic array
        self.target_direction = 0  # radians

    def beamform_audio(self, multi_channel_audio):
        """Apply beamforming to focus on speaker direction"""
        # Calculate steering delays
        delays = []
        for pos in self.mic_positions:
            delay = self.calculate_delay(pos, self.target_direction)
            delays.append(delay)

        # Apply delays and sum channels
        delayed_audio = []
        for i, audio in enumerate(multi_channel_audio):
            delayed = self.apply_delay(audio, delays[i])
            delayed_audio.append(delayed)

        # Sum all channels
        beamformed = np.sum(delayed_audio, axis=0)
        return beamformed

    def noise_reduction(self, audio_signal):
        """Apply noise reduction using spectral subtraction"""
        # Convert to frequency domain
        fft_signal = np.fft.fft(audio_signal)

        # Estimate noise spectrum (first 100 samples assumed to be noise)
        noise_samples = audio_signal[:100]
        noise_fft = np.fft.fft(noise_samples)
        noise_spectrum = np.abs(noise_fft)**2

        # Apply spectral subtraction
        signal_spectrum = np.abs(fft_signal)**2
        enhanced_spectrum = np.maximum(signal_spectrum - noise_spectrum, 0.1 * signal_spectrum)

        # Convert back to time domain
        enhanced_signal = np.real(np.fft.ifft(enhanced_spectrum * np.exp(1j * np.angle(fft_signal))))

        return enhanced_signal.astype(np.float32)
```

## Context-Aware Processing

### Conversation Context
```python
class ContextManager:
    def __init__(self):
        self.conversation_history = []
        self.current_task = None
        self.robot_state = {}
        self.environment_state = {}

    def update_context(self, command_result=None):
        """Update context based on command result"""
        if command_result:
            self.conversation_history.append({
                'timestamp': rospy.Time.now(),
                'result': command_result,
                'feedback': command_result.feedback
            })

    def resolve_pronouns(self, text, context=None):
        """Resolve pronouns in the context of recent conversation"""
        if not context:
            context = self.conversation_history[-5:]  # Last 5 exchanges

        # Simple pronoun resolution
        text_resolved = text.replace("it", self.get_recent_object())
        text_resolved = text_resolved.replace("there", self.get_recent_location())

        return text_resolved

    def get_recent_object(self):
        """Get most recently referenced object"""
        for entry in reversed(self.conversation_history):
            if 'object' in entry.get('entities', {}):
                return entry['entities']['object']
        return "object"
```

## Error Handling and Recovery

### Clarification Requests
```python
class ErrorHandler:
    def __init__(self):
        self.uncertainty_threshold = 0.3
        self.confusion_phrases = [
            "I'm sorry, I didn't understand that.",
            "Could you please repeat that?",
            "I didn't catch that, could you say it again?",
            "I'm not sure what you mean, could you be more specific?"
        ]

    def handle_uncertain_recognition(self, confidence, recognized_text):
        """Handle cases where recognition confidence is low"""
        if confidence < self.uncertainty_threshold:
            # Ask for clarification
            self.request_clarification(recognized_text)
            return False
        return True

    def request_clarification(self, ambiguous_command):
        """Request user to clarify ambiguous command"""
        # Generate specific clarification request
        if self.is_navigation_ambiguous(ambiguous_command):
            clarification = "I heard you say '{}', but I'm not sure about the destination. Could you specify where you'd like me to go?" .format(ambiguous_command)
        elif self.is_object_ambiguous(ambiguous_command):
            clarification = "I heard '{}', but I don't see that object nearby. Could you point it out or describe it differently?".format(ambiguous_command)
        else:
            clarification = np.random.choice(self.confusion_phrases)

        self.speak_response(clarification)
```

## Performance Optimization

### Model Optimization
- **Quantization**: Reduce model size for faster inference
- **Caching**: Cache common transcriptions
- **Streaming**: Process audio in chunks rather than full utterances
- **Early stopping**: Stop processing when confidence is high

### Resource Management
```python
class ResourceManager:
    def __init__(self):
        self.gpu_memory_fraction = 0.5
        self.cpu_affinity = [0, 1]  # Use specific CPU cores
        self.max_concurrent_tasks = 2

    def optimize_whisper_inference(self):
        """Optimize Whisper inference for real-time performance"""
        # Use mixed precision if available
        if torch.cuda.is_available() and torch.cuda.get_device_capability()[0] >= 7:
            torch.backends.cudnn.benchmark = True

        # Set memory fraction to prevent GPU memory issues
        if torch.cuda.is_available():
            torch.cuda.set_per_process_memory_fraction(self.gpu_memory_fraction)
```

## Best Practices

### 1. Robustness
- Handle various accents and speaking speeds
- Implement confidence scoring for uncertain recognitions
- Provide fallback mechanisms for failed recognitions
- Use context to disambiguate similar-sounding commands

### 2. Privacy
- Implement local processing where possible
- Encrypt sensitive audio data
- Provide clear privacy notices
- Allow users to delete audio history

### 3. Accessibility
- Support multiple languages
- Provide visual feedback for voice commands
- Allow adjustment of sensitivity and speed
- Support alternative input methods

### 4. Integration
- Seamless integration with ROS 2 navigation
- Proper error handling and recovery
- Clear separation of speech recognition and action execution
- Comprehensive logging and debugging capabilities

## Troubleshooting Common Issues

### 1. Recognition Accuracy
- Background noise interference
- Microphone quality issues
- Speaker accent variations
- Audio format compatibility problems

### 2. Performance
- Processing latency issues
- GPU memory limitations
- CPU resource contention
- Real-time processing constraints

### 3. Integration Problems
- Audio format mismatches
- ROS message type incompatibilities
- Timing synchronization issues
- Coordinate frame mismatches

### 4. Context Management
- Pronoun resolution failures
- Conversation state tracking
- Multi-user interaction handling
- Task interruption and resumption

Voice-to-Action systems provide a natural interface for human-robot interaction, enabling more intuitive and accessible control of humanoid robots through spoken commands.