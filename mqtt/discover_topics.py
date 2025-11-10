#!/usr/bin/env python3
"""
MQTT Topic Discovery Script
Discovers all available topics on the MQTT broker, especially those related to bioreactor_sim
"""

import json
import os
import time
from datetime import datetime
from collections import defaultdict
import paho.mqtt.client as mqtt

# MQTT Configuration
MQTT_BROKER = os.getenv("MQTT_BROKER", "engf0001.cs.ucl.ac.uk")
MQTT_PORT = int(os.getenv("MQTT_PORT", "1883"))
MQTT_USERNAME = os.getenv("MQTT_USERNAME", None)
MQTT_PASSWORD = os.getenv("MQTT_PASSWORD", None)

# Track discovered topics and message counts
discovered_topics = defaultdict(int)


def on_connect(client, userdata, flags, rc):
    """Callback for when the client connects to the broker."""
    if rc == 0:
        print(f"✓ Connected to MQTT broker at {MQTT_BROKER}:{MQTT_PORT}")
        print(f"  Connection flags: {flags}")
        print("-" * 60)
        
        # Subscribe to ALL topics using wildcard
        all_topics_pattern = "#"
        
        print(f"Subscribing to ALL topics: {all_topics_pattern}")
        result, mid = client.subscribe(all_topics_pattern)
        if result == mqtt.MQTT_ERR_SUCCESS:
            print(f"  ✓ Subscribed to: {all_topics_pattern} (mid: {mid})")
        else:
            print(f"  ✗ Failed to subscribe to: {all_topics_pattern} (error: {result})")
        
        print("\nWill print ALL messages from ANY discovered topic...")
        print("Press Ctrl+C to stop")
        print("=" * 60)
        
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


# Add debug print to confirm message callback is triggered
def on_message(client, userdata, msg):
    print("Message received callback triggered")  # Debug print
    topic = msg.topic
    timestamp = datetime.now().isoformat()
    
    # Track this topic
    is_new_topic = topic not in discovered_topics
    discovered_topics[topic] += 1
    message_num = discovered_topics[topic]
    
    # Print message header
    print(f"\n{'='*60}")
    if is_new_topic:
        print(f"📨 NEW TOPIC DISCOVERED: {topic}")
    else:
        print(f"📨 MESSAGE #{message_num} from: {topic}")
    print(f"{'='*60}")
    print(f"Timestamp: {timestamp}")
    print(f"QoS: {msg.qos}")
    print(f"Retain: {msg.retain}")
    print(f"Payload size: {len(msg.payload)} bytes")
    
    # Try to decode and print payload
    try:
        raw_payload = msg.payload.decode('utf-8')
        print(f"\nPayload (raw):")
        print("-" * 60)
        
        # Try to parse as JSON
        try:
            payload = json.loads(raw_payload)
            print("Format: JSON")
            print("\nContent:")
            print(json.dumps(payload, indent=2))
        except json.JSONDecodeError:
            # Not JSON, print as text
            print("Format: Text/Other")
            print("\nContent:")
            print(raw_payload)
            
    except UnicodeDecodeError:
        print("Format: Binary (non-UTF-8)")
        print(f"\nContent (hex): {msg.payload.hex()[:200]}...")
    
    print(f"\nTotal messages from this topic: {message_num}")
    print(f"{'='*60}\n")


def print_summary():
    """Print summary of discovered topics."""
    print("\n" + "=" * 60)
    print("DISCOVERY SUMMARY")
    print("=" * 60)
    
    if not discovered_topics:
        print("\n⚠ No topics discovered. Possible reasons:")
        print("  - No messages are being published")
        print("  - Topic names don't match the patterns")
        print("  - Broker requires authentication")
        return
    
    # Group topics by stream
    streams = defaultdict(list)
    for topic in discovered_topics.keys():
        parts = topic.split('/')
        if len(parts) >= 2:
            stream_name = parts[1]
            streams[stream_name].append(topic)
        else:
            streams['other'].append(topic)
    
    print(f"\nTotal unique topics discovered: {len(discovered_topics)}")
    print(f"Total messages received: {sum(discovered_topics.values())}")
    
    print("\n" + "-" * 60)
    print("TOPICS BY STREAM:")
    print("-" * 60)
    
    for stream_name in sorted(streams.keys()):
        stream_topics = streams[stream_name]
        print(f"\n📊 Stream: {stream_name}")
        for topic in sorted(stream_topics):
            count = discovered_topics[topic]
            print(f"  • {topic}")
            print(f"    Messages: {count}")
    
    print("\n" + "=" * 60)
    print("📋 ALL STREAM NAMES FOUND:")
    print("=" * 60)
    
    # Extract all unique stream names from topics
    all_stream_names = set()
    bioreactor_streams = set()
    
    for topic in discovered_topics.keys():
        parts = topic.split('/')
        if len(parts) >= 2:
            stream_name = parts[1]
            all_stream_names.add(stream_name)
            # Check if it's a bioreactor_sim topic
            if 'bioreactor_sim' in topic:
                bioreactor_streams.add(stream_name)
    
    if bioreactor_streams:
        print("\n✅ BIOREACTOR SIMULATOR STREAMS:")
        print("-" * 60)
        for stream in sorted(bioreactor_streams):
            # Find example topics for this stream
            example_topics = [t for t in discovered_topics.keys() if stream in t.split('/')]
            print(f"  • {stream}")
            for topic in sorted(example_topics)[:3]:  # Show first 3 example topics
                print(f"      → {topic}")
            if len(example_topics) > 3:
                print(f"      ... and {len(example_topics) - 3} more topics")
    
    print("\n" + "-" * 60)
    print("📊 ALL STREAM NAMES (from any topic):")
    print("-" * 60)
    if all_stream_names:
        for stream in sorted(all_stream_names):
            if stream != 'other':
                print(f"  • {stream}")
    else:
        print("  (none found)")
    
    # Try to identify telemetry/summary specific streams
    print("\n" + "-" * 60)
    print("🔍 TELEMETRY/SUMMARY STREAMS:")
    print("-" * 60)
    telemetry_streams = set()
    for topic in discovered_topics.keys():
        if '/telemetry/summary' in topic:
            parts = topic.split('/')
            if len(parts) >= 2:
                telemetry_streams.add(parts[1])
    
    if telemetry_streams:
        print("\nFound streams with telemetry/summary topics:")
        for stream in sorted(telemetry_streams):
            print(f"  ✓ {stream}")
            # Find the exact topic
            matching_topics = [t for t in discovered_topics.keys() if f'/{stream}/telemetry/summary' in t]
            for topic in matching_topics:
                print(f"    Topic: {topic}")
    else:
        print("\n⚠ No 'telemetry/summary' topics found.")
        print("\n💡 TIP: Look at the topics above to identify the correct stream names.")
        print("   Stream names appear as the second part of the topic path.")
        print("   Example: 'bioreactor_sim/STREAM_NAME/telemetry/summary'")
    
    # Extract stream names for easy reference
    all_stream_names = set()
    bioreactor_streams = set()
    for topic in discovered_topics.keys():
        parts = topic.split('/')
        if len(parts) >= 2:
            stream_name = parts[1]
            all_stream_names.add(stream_name)
            if 'bioreactor_sim' in topic:
                bioreactor_streams.add(stream_name)
    
    # Save results to file
    results_file = "discovered_topics.json"
    results = {
        "timestamp": datetime.now().isoformat(),
        "broker": f"{MQTT_BROKER}:{MQTT_PORT}",
        "total_topics": len(discovered_topics),
        "total_messages": sum(discovered_topics.values()),
        "stream_names": {
            "all_streams": sorted(all_stream_names),
            "bioreactor_streams": sorted(bioreactor_streams),
            "telemetry_summary_streams": sorted([s for topic in discovered_topics.keys() 
                                                 if '/telemetry/summary' in topic 
                                                 and (parts := topic.split('/')) and len(parts) >= 2 
                                                 for s in [parts[1]]])
        },
        "topics": {
            topic: {
                "message_count": count
            }
            for topic, count in discovered_topics.items()
        }
    }
    
    with open(results_file, 'w') as f:
        json.dump(results, f, indent=2)
    
    print(f"\n✓ Results saved to: {results_file}")
    print(f"\n📝 QUICK REFERENCE - Stream names to use in your logger:")
    print("-" * 60)
    if bioreactor_streams:
        print("   Add these to DATA_STREAMS in mqtt_logger.py:")
        for stream in sorted(bioreactor_streams):
            print(f'     "{stream}",')
    else:
        print("   No bioreactor streams found. Check the topics above.")


def main():
    """Main function to discover MQTT topics."""
    print("MQTT Topic Discovery Tool")
    print("=" * 60)
    print(f"Broker: {MQTT_BROKER}:{MQTT_PORT}")
    print(f"Subscribing to ALL topics (#)")
    print("-" * 60)
    
    # Create MQTT client with unique client ID
    client_id = f"topic_discovery_{int(time.time())}"
    client = mqtt.Client(client_id=client_id)
    
    # Set callbacks
    client.on_connect = on_connect
    client.on_message = on_message
    client.on_subscribe = on_subscribe
    
    # Set credentials if provided
    if MQTT_USERNAME and MQTT_PASSWORD:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    
    # Enable Paho MQTT logging for detailed debugging
    client.enable_logger()
    
    try:
        # Connect to broker
        print(f"Connecting to {MQTT_BROKER}:{MQTT_PORT}...")
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        
        # Run continuously until interrupted
        print("\nListening for messages... (Press Ctrl+C to stop)\n")
        client.loop_forever()
        
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("STOPPED BY USER")
        print("=" * 60)
        client.loop_stop()
        client.disconnect()
        print_summary()
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        client.disconnect()


if __name__ == "__main__":
    main()

