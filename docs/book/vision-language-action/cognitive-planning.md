---
title: Cognitive Planning
sidebar_position: 3
description: Using LLMs to translate natural language to robot action sequences
---

# Cognitive Planning: Using LLMs to Translate Natural Language to Robot Action Sequences

## Overview

Cognitive planning represents the intersection of large language models (LLMs) and robotics, where natural language commands are translated into executable sequences of robot actions. This module explores how LLMs can be used to bridge the gap between human intention and robot behavior, enabling more intuitive human-robot interaction.

## Key Concepts

### Language-to-Action Translation
- Natural language understanding for robot commands
- Task decomposition from high-level instructions
- Context-aware action selection
- Execution monitoring and error recovery

### LLM Integration Architecture
- Prompt engineering for robot control
- Chain-of-thought reasoning for complex tasks
- Multi-modal reasoning combining language and perception
- Feedback integration for adaptive planning

## Cognitive Planning Architecture

### System Components
```
Natural Language Input → LLM Processing → Task Decomposition → Action Sequencing → Execution → Feedback → Adaptation
```

### Core Components
- **Language Understanding**: Interpret natural language commands
- **World Modeling**: Maintain representation of environment and robot state
- **Planning Engine**: Generate action sequences from high-level goals
- **Execution Monitor**: Track execution and handle deviations
- **Learning Module**: Adapt planning based on experience

## LLM Integration Approaches

### Direct Command Translation
```python
import openai
import json
from typing import List, Dict, Any

class DirectTranslationPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key
        self.system_prompt = """
        You are a robot action planner. Your role is to convert natural language commands
        into sequences of robot actions. Respond in JSON format with an array of actions.

        Available actions:
        - NAVIGATE_TO(location): Move robot to specified location
        - DETECT_OBJECT(object_type): Look for objects of specified type
        - GRASP_OBJECT(object_id): Pick up a specific object
        - PLACE_OBJECT(destination): Place held object at destination
        - SAY(text): Speak text aloud
        - WAIT(duration): Wait for specified duration

        Example:
        Input: "Go to the kitchen and bring me a cup"
        Output: [
            {"action": "NAVIGATE_TO", "params": {"location": "kitchen"}},
            {"action": "DETECT_OBJECT", "params": {"object_type": "cup"}},
            {"action": "GRASP_OBJECT", "params": {"object_id": "cup_123"}},
            {"action": "NAVIGATE_TO", "params": {"location": "user"}},
            {"action": "PLACE_OBJECT", "params": {"destination": "table"}}
        ]
        """

    def plan_actions(self, command: str, robot_state: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Plan robot actions from natural language command"""
        prompt = f"""
        Current robot state: {json.dumps(robot_state)}
        Command: {command}

        Plan the sequence of actions to execute this command.
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "system", "content": self.system_prompt},
                {"role": "user", "content": prompt}
            ],
            temperature=0.1,
            max_tokens=1000
        )

        # Parse and validate the response
        plan = self.parse_plan(response.choices[0].message.content)
        return self.validate_plan(plan)
```

### Chain-of-Thought Reasoning
```python
class ChainOfThoughtPlanner:
    def __init__(self, api_key: str):
        openai.api_key = api_key

    def plan_with_reasoning(self, command: str, environment_state: Dict[str, Any]) -> Dict[str, Any]:
        """Plan actions with explicit reasoning steps"""
        prompt = f"""
        Command: {command}
        Environment: {json.dumps(environment_state)}

        Think step by step about how to accomplish this task:

        1. What is the goal?
        2. What are the current conditions?
        3. What are the intermediate steps?
        4. What potential obstacles might arise?
        5. What is the final plan?

        Then provide the action sequence in JSON format.
        """

        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.3
        )

        return self.extract_plan_and_reasoning(response.choices[0].message.content)
```

## Task Decomposition Strategies

### Hierarchical Task Networks (HTNs)
```python
class HTNPlanner:
    def __init__(self):
        self.primitive_actions = {
            'navigate': self.execute_navigate,
            'detect': self.execute_detect,
            'grasp': self.execute_grasp,
            'place': self.execute_place,
            'say': self.execute_say
        }

        self.compound_tasks = {
            'bring_object': self.decompose_bring_object,
            'clean_area': self.decompose_clean_area,
            'assist_person': self.decompose_assist_person
        }

    def decompose_bring_object(self, obj_type: str, destination: str, current_pos: str) -> List[Dict[str, Any]]:
        """Decompose 'bring object' task into primitive actions"""
        return [
            {"action": "navigate", "params": {"location": self.find_location(obj_type)}},
            {"action": "detect", "params": {"object_type": obj_type}},
            {"action": "grasp", "params": {"object_type": obj_type}},
            {"action": "navigate", "params": {"location": destination}},
            {"action": "place", "params": {"destination": destination}}
        ]

    def plan_from_high_level(self, task: str) -> List[Dict[str, Any]]:
        """Plan from high-level task description"""
        if task.startswith("bring"):
            # Extract object and destination
            obj_type = self.extract_object_type(task)
            destination = self.extract_destination(task)
            return self.decompose_bring_object(obj_type, destination, self.current_position)
        else:
            # Fallback to LLM for complex tasks
            return self.llm_fallback_plan(task)
```

### Symbolic Planning Integration
```python
class SymbolicPlanner:
    def __init__(self):
        # Define predicates and operators for symbolic planning
        self.predicates = {
            'at(robot, location)',
            'holding(robot, object)',
            'free(hand)',
            'connected(loc1, loc2)'
        }

        self.operators = {
            'navigate': {
                'preconditions': ['at(robot, from_loc)', 'connected(from_loc, to_loc)'],
                'effects': ['at(robot, to_loc)', '~at(robot, from_loc)']
            },
            'grasp': {
                'preconditions': ['at(robot, object_loc)', 'free(hand)', 'at(object, object_loc)'],
                'effects': ['holding(robot, object)', '~free(hand)']
            }
        }

    def integrate_symbolic_planning(self, llm_plan: List[Dict[str, Any]]) -> List[Dict[str, Any]]:
        """Integrate symbolic planning with LLM-generated plan"""
        # Validate LLM plan against symbolic constraints
        validated_plan = []
        current_state = self.initial_state

        for action in llm_plan:
            if self.check_preconditions(action, current_state):
                validated_plan.append(action)
                current_state = self.update_state(current_state, action)
            else:
                # Replan using symbolic planner for this step
                corrected_action = self.symbolic_replan(action, current_state)
                validated_plan.append(corrected_action)
                current_state = self.update_state(current_state, corrected_action)

        return validated_plan
```

## Multi-Modal Integration

### Vision-Language Integration
```python
class MultiModalPlanner:
    def __init__(self):
        self.vision_module = VisionModule()
        self.language_module = LanguageModule()
        self.planning_module = PlanningModule()

    def multimodal_plan(self, command: str, visual_input: Any) -> List[Dict[str, Any]]:
        """Plan using both language and visual input"""
        # Extract relevant objects from visual input
        detected_objects = self.vision_module.detect_objects(visual_input)

        # Parse language command
        parsed_command = self.language_module.parse_command(command)

        # Combine visual and linguistic information
        planning_context = {
            'command': parsed_command,
            'objects': detected_objects,
            'environment': self.get_environment_state(),
            'robot_capabilities': self.get_robot_capabilities()
        }

        # Generate plan based on multi-modal context
        plan = self.planning_module.generate_plan(planning_context)

        return plan

    def handle_ambiguous_reference(self, command: str, visual_input: Any) -> List[Dict[str, Any]]:
        """Handle cases where language refers to objects not clearly visible"""
        detected_objects = self.vision_module.detect_objects(visual_input)
        referred_object = self.language_module.extract_object_reference(command)

        if not self.is_object_visible(referred_object, detected_objects):
            # Request clarification or search for object
            if self.has_multiple_candidates(referred_object, detected_objects):
                return self.request_disambiguation(referred_object, detected_objects)
            else:
                return self.search_for_object(referred_object)

        return self.multimodal_plan(command, visual_input)
```

## Context Awareness and Memory

### Episodic Memory Integration
```python
class ContextAwarePlanner:
    def __init__(self):
        self.episodic_memory = EpisodicMemory()
        self.semantic_memory = SemanticMemory()
        self.working_memory = WorkingMemory()

    def contextual_plan(self, command: str) -> List[Dict[str, Any]]:
        """Plan considering past episodes and context"""
        # Retrieve relevant past episodes
        relevant_episodes = self.episodic_memory.retrieve_similar_episodes(command)

        # Get semantic knowledge about the task
        semantic_knowledge = self.semantic_memory.get_knowledge(command)

        # Combine with current working memory
        context = {
            'past_experience': relevant_episodes,
            'semantic_knowledge': semantic_knowledge,
            'current_state': self.working_memory.get_current_state(),
            'social_context': self.get_social_context()
        }

        # Generate plan considering context
        plan = self.generate_contextual_plan(command, context)

        return plan

    def update_memory(self, plan: List[Dict[str, Any]], outcome: str):
        """Update memory based on plan execution outcome"""
        episode = {
            'input': self.last_command,
            'plan': plan,
            'outcome': outcome,
            'context': self.working_memory.get_episode_context()
        }

        self.episodic_memory.store(episode)

        # Update semantic memory with learned patterns
        self.semantic_memory.update_from_experience(episode)
```

## Error Handling and Recovery

### Plan Monitoring and Correction
```python
class PlanMonitor:
    def __init__(self):
        self.execution_trace = []
        self.deviation_threshold = 0.1
        self.recovery_strategies = {
            'retry': self.retry_action,
            'skip': self.skip_and_continue,
            'replan': self.generate_new_plan,
            'ask_help': self.request_human_assistance
        }

    def monitor_execution(self, current_action: Dict[str, Any], expected_state: Dict[str, Any]) -> Dict[str, Any]:
        """Monitor plan execution and detect deviations"""
        actual_state = self.get_current_robot_state()

        deviation = self.calculate_deviation(expected_state, actual_state)

        if deviation > self.deviation_threshold:
            # Deviation detected, trigger recovery
            recovery_action = self.select_recovery_strategy(
                current_action,
                expected_state,
                actual_state,
                deviation
            )

            return recovery_action
        else:
            # Execution proceeding as expected
            self.execution_trace.append({
                'action': current_action,
                'expected_state': expected_state,
                'actual_state': actual_state,
                'deviation': deviation
            })

            return None  # Continue normal execution

    def calculate_deviation(self, expected: Dict[str, Any], actual: Dict[str, Any]) -> float:
        """Calculate deviation between expected and actual states"""
        deviation_score = 0.0

        # Location deviation
        if 'location' in expected and 'location' in actual:
            deviation_score += self.spatial_deviation(expected['location'], actual['location'])

        # Object state deviation
        if 'held_object' in expected and 'held_object' in actual:
            deviation_score += self.object_state_deviation(expected['held_object'], actual['held_object'])

        # Task completion deviation
        if 'task_progress' in expected and 'task_progress' in actual:
            deviation_score += abs(expected['task_progress'] - actual['task_progress'])

        return deviation_score
```

## Learning and Adaptation

### Plan Improvement Through Experience
```python
class AdaptivePlanner:
    def __init__(self):
        self.success_rate_tracker = {}
        self.failure_patterns = {}
        self.plan_improvement_engine = PlanImprovementEngine()

    def learn_from_execution(self, plan: List[Dict[str, Any]], outcome: str, context: Dict[str, Any]):
        """Learn from plan execution results"""
        if outcome == 'SUCCESS':
            self.record_success(plan, context)
        else:
            self.analyze_failure(plan, outcome, context)
            self.generate_improvement_plan(plan, outcome, context)

    def record_success(self, plan: List[Dict[str, Any]], context: Dict[str, Any]):
        """Record successful plan execution"""
        for action in plan:
            action_key = action['action']
            if action_key not in self.success_rate_tracker:
                self.success_rate_tracker[action_key] = {'success': 0, 'total': 0}

            self.success_rate_tracker[action_key]['success'] += 1
            self.success_rate_tracker[action_key]['total'] += 1

    def analyze_failure(self, plan: List[Dict[str, Any]], failure_type: str, context: Dict[str, Any]):
        """Analyze failure patterns"""
        failure_signature = self.extract_failure_signature(plan, failure_type, context)

        if failure_signature not in self.failure_patterns:
            self.failure_patterns[failure_signature] = {
                'frequency': 0,
                'contexts': [],
                'recommended_solutions': []
            }

        self.failure_patterns[failure_signature]['frequency'] += 1
        self.failure_patterns[failure_signature]['contexts'].append(context)

    def improve_plan(self, original_plan: List[Dict[str, Any]], context: Dict[str, Any]) -> List[Dict[str, Any]]:
        """Improve plan based on learned patterns"""
        improved_plan = []

        for action in original_plan:
            # Check if this action type has improvement suggestions
            action_key = action['action']
            if action_key in self.plan_improvement_engine.improvements:
                improved_action = self.plan_improvement_engine.apply_improvement(
                    action, context
                )
                improved_plan.append(improved_action)
            else:
                improved_plan.append(action)

        return improved_plan
```

## Safety and Ethics Considerations

### Safety Constraints Integration
```python
class SafePlanner:
    def __init__(self):
        self.safety_constraints = {
            'human_safety': self.check_human_safety,
            'property_protection': self.check_property_safety,
            'robot_safety': self.check_robot_safety,
            'social_norms': self.check_social_norms
        }

    def generate_safe_plan(self, command: str) -> List[Dict[str, Any]]:
        """Generate plan that respects safety constraints"""
        initial_plan = self.llm_plan(command)

        # Validate each action against safety constraints
        safe_plan = []
        for action in initial_plan:
            if self.is_action_safe(action):
                safe_plan.append(action)
            else:
                # Generate safe alternative
                safe_alternative = self.generate_safe_alternative(action)
                if safe_alternative:
                    safe_plan.append(safe_alternative)
                else:
                    # Cannot safely execute this part of the plan
                    raise UnsafePlanError(f"Cannot safely execute action: {action}")

        return safe_plan

    def check_human_safety(self, action: Dict[str, Any], context: Dict[str, Any]) -> bool:
        """Check if action is safe for humans in environment"""
        if action['action'] == 'navigate':
            target_location = action['params']['location']
            humans_nearby = self.get_humans_in_proximity(target_location)
            return self.would_approach_humans_safely(humans_nearby, action)
        elif action['action'] == 'grasp':
            object_props = self.get_object_properties(action['params']['object_id'])
            return not self.object_is_dangerous(object_props)

        return True  # Default to safe for other actions
```

## Performance Optimization

### Caching and Efficiency
```python
class EfficientPlanner:
    def __init__(self):
        self.plan_cache = PlanCache(max_size=1000)
        self.pattern_recognizer = PatternRecognizer()
        self.computation_scheduler = ComputationScheduler()

    def plan_efficiently(self, command: str) -> List[Dict[str, Any]]:
        """Plan efficiently using caching and pattern recognition"""
        # Check if command matches cached pattern
        cached_result = self.plan_cache.lookup(command)
        if cached_result:
            # Adapt cached plan to current context
            return self.adapt_cached_plan(cached_result, self.get_current_context())

        # Recognize pattern in command
        pattern_match = self.pattern_recognizer.match_pattern(command)
        if pattern_match:
            # Generate plan based on recognized pattern
            plan = self.generate_pattern_based_plan(pattern_match, self.get_current_context())
        else:
            # Use LLM for novel commands
            plan = self.llm_plan(command)

        # Cache the result
        self.plan_cache.store(command, plan)

        return plan
```

## Best Practices

### 1. Robustness
- Implement multiple planning strategies
- Use confidence scores for plan selection
- Provide fallback mechanisms
- Validate plans before execution

### 2. Interpretability
- Explain planning decisions
- Provide step-by-step reasoning
- Allow user intervention
- Log planning process

### 3. Efficiency
- Cache frequently used plans
- Use pattern recognition
- Optimize LLM queries
- Parallelize planning components

### 4. Safety
- Integrate safety constraints
- Monitor plan execution
- Implement recovery procedures
- Consider ethical implications

## Troubleshooting Common Issues

### 1. Planning Failures
- Handle ambiguous commands
- Manage incomplete world knowledge
- Deal with changing environments
- Address computational limitations

### 2. Integration Problems
- Synchronize language and perception
- Handle timing constraints
- Manage communication latencies
- Coordinate multiple systems

### 3. Performance Issues
- Optimize LLM usage
- Manage computational resources
- Balance quality and speed
- Handle real-time requirements

### 4. Safety Concerns
- Validate action safety
- Handle unexpected situations
- Implement emergency procedures
- Consider social implications

Cognitive planning enables humanoid robots to understand and execute complex natural language commands by leveraging the reasoning capabilities of large language models while maintaining safety and reliability.