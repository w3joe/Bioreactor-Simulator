#!/usr/bin/env python3
"""
MQTT Data Logger for Bioreactor Simulator
Subscribes to bioreactor_sim/<DATA_STREAM>/telemetry/summary topics
and logs received data to a JSON file.
"""

import json
import os
from datetime import datetime
from typing import Dict
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "engf0001.cs.ucl.ac.uk")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", None)
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", None)

# Data streams to subscribe to
DATA_STREAMS = [
    "nofaults",
    "single_fault",
    "three_faults",
    "variable_setpoints"
]

# Get the directory where this script is located
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# Output files - one for each stream
def get_output_file(data_stream: str) -> str:
    """Get the output file path for a given data stream."""
    filename = f"{data_stream}_log.json"
    return os.path.join(SCRIPT_DIR, filename)

# Store message counts per stream
message_counts: Dict[str, int] = {stream: 0 for stream in DATA_STREAMS}


def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        print(f"✓ Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        print(f"  Connection flags: {flags}")
        print(f"  Waiting for messages...")
        print("-" * 50)

        # Subscribe to all topics under bioreactor_sim
        topic_pattern = "bioreactor_sim/#"
        result, mid = client.subscribe(topic_pattern)
        if result == mqtt.MQTT_ERR_SUCCESS:
            print(f"✓ Subscribed to: {topic_pattern} (mid: {mid})")
        else:
            print(f"✗ Failed to subscribe to: {topic_pattern} (error: {result})")
    else:
        error_messages = {
            1: "incorrect protocol version",
            2: "invalid client identifier",
            3: "server unavailable",
            4: "bad username or password",
            5: "not authorised"
        }
        error_msg = error_messages.get(rc, f"unknown error ({rc})")
        print(f"✗ Failed to connect to broker. Return code {rc}: {error_msg}")


def on_subscribe(client, userdata, mid, granted_qos):
    """Callback for when a subscription is confirmed."""
    print(f"✓ Subscription confirmed (mid: {mid}, QoS: {granted_qos})")


def on_disconnect(client, userdata, rc):
    """Callback for when the client disconnects."""
    if rc != 0:
        print(f"✗ Unexpected disconnection from broker (rc: {rc})")
    else:
        print(f"✓ Disconnected from broker")


def on_message(client, userdata, msg):
    """Callback for when a message is received."""
    # Log raw message reception
    raw_payload = msg.payload.decode('utf-8')
    payload_size = len(msg.payload)
    timestamp = datetime.now().isoformat()
    
    print(f"\n{'='*50}")
    print(f"MESSAGE RECEIVED at {timestamp}")
    print(f"Topic: {msg.topic}")
    print(f"Payload size: {payload_size} bytes")
    print(f"QoS: {msg.qos}")
    print(f"Retain: {msg.retain}")
    
    # Extract data stream from topic
    topic_parts = msg.topic.split('/')
    data_stream = topic_parts[1] if len(topic_parts) > 1 else "unknown"
    print(f"   Detected stream: {data_stream}")
    
    try:
        # Parse the received JSON message
        payload = json.loads(raw_payload)
        
        # Log payload structure
        if isinstance(payload, dict):
            print(f"   Payload keys: {list(payload.keys())}")
            # Show sample values (first few keys)
            sample_keys = list(payload.keys())[:5]
            for key in sample_keys:
                value = payload[key]
                if isinstance(value, (int, float, str, bool)):
                    print(f"     {key}: {value}")
                else:
                    print(f"     {key}: {type(value).__name__}")
        else:
            print(f"   Payload type: {type(payload).__name__}")
        
        # Create log entry
        log_entry = {
            "timestamp": timestamp,
            "topic": msg.topic,
            "data_stream": data_stream,
            "data": payload
        }
        
        # Increment message count for this stream
        if data_stream in message_counts:
            message_counts[data_stream] += 1
            print(f"   Message count for {data_stream}: {message_counts[data_stream]}")
        
        # Write to file immediately (append mode)
        write_to_file(log_entry, data_stream)
        
        print(f"✓ Successfully processed message from {data_stream}")
        
    except json.JSONDecodeError as e:
        print(f"✗ ERROR: Failed to decode JSON from {msg.topic}")
        print(f"   Error: {e}")
        print(f"   Raw payload (first 200 chars): {raw_payload[:200]}")
    except Exception as e:
        print(f"✗ ERROR: Failed to process message from {msg.topic}")
        print(f"   Error: {e}")
        import traceback
        traceback.print_exc()
    print(f"{'='*50}\n")


def write_to_file(entry: Dict, data_stream: str):
    """Write a log entry to the appropriate JSON file based on data stream."""
    try:
        output_file = get_output_file(data_stream)
        
        # Read existing data if file exists
        if os.path.exists(output_file):
            try:
                with open(output_file, 'r') as f:
                    existing_data = json.load(f)
            except json.JSONDecodeError as e:
                print(f"Warning: Could not parse existing JSON in {output_file}: {e}. Starting fresh.")
                existing_data = []
            except Exception as e:
                print(f"Warning: Error reading {output_file}: {e}. Starting fresh.")
                existing_data = []
        else:
            existing_data = []
        
        # Append new entry
        existing_data.append(entry)
        
        # Write back to file
        with open(output_file, 'w') as f:
            json.dump(existing_data, f, indent=2)
        
        # Debug output
        print(f"   ✓ Written to: {output_file} (total entries: {len(existing_data)})")
            
    except PermissionError as e:
        print(f"ERROR: Permission denied writing to file for {data_stream}: {e}")
        print(f"  File path: {output_file}")
    except Exception as e:
        print(f"ERROR: Error writing to file for {data_stream}: {e}")
        print(f"  File path: {output_file}")
        import traceback
        traceback.print_exc()


def main():
    """Main function to set up and run the MQTT client."""
    print("Starting MQTT Data Logger...")
    print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"Script directory: {SCRIPT_DIR}")
    print(f"Subscribing to {len(DATA_STREAMS)} data streams")
    print("\nOutput files:")
    for stream in DATA_STREAMS:
        filepath = get_output_file(stream)
        print(f"  - {stream}: {filepath}")
    print("-" * 50)
    
    # Create MQTT client
    client = mqtt.Client()
    
    # Set callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    client.on_disconnect = on_disconnect
    
    # Set credentials if provided
    if MQTT_USERNAME and MQTT_PASSWORD:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    
    try:
        # Connect to broker
        print(f"Attempting to connect to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Start the loop to process network traffic
        print("Starting MQTT loop (waiting for messages)...")
        print("Press Ctrl+C to stop\n")
        client.loop_forever()
        
    except KeyboardInterrupt:
        print("\nShutting down...")
        client.disconnect()
        print("\nMessage counts per stream:")
        total = 0
        for stream in DATA_STREAMS:
            count = message_counts[stream]
            total += count
            print(f"  - {stream}: {count} messages -> {get_output_file(stream)}")
        print(f"\nTotal: {total} messages logged")
    except Exception as e:
        print(f"Error: {e}")
        client.disconnect()


if __name__ == "__main__":
    main()


