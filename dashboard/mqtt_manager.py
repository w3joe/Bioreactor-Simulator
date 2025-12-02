import queue
import streamlit as st
import paho.mqtt.client as mqtt
import pandas as pd
import json
import ssl
import time

#create a safe queue to get data from MQTT
incoming_mqtt_queue = queue.Queue()

def initialize_session_state():
    if "current_stirring_rpm" not in st.session_state:
        st.session_state.current_stirring_rpm = 0
    if "target_stirring_rpm" not in st.session_state:
        st.session_state.target_stirring_rpm = 0
    if "stirring_status" not in st.session_state:
        st.session_state.stirring_status = "🟠"

    if "current_heating_temp" not in st.session_state:
        st.session_state.current_heating_temp = 20.0
    if "target_heating_temp" not in st.session_state:
        st.session_state.target_heating_temp = 20.0
    if "heating_status" not in st.session_state:
        st.session_state.heating_status = "🟠" 

    if "current_ph_value" not in st.session_state:
        st.session_state.current_ph_value = 7.0
    if "target_ph_value" not in st.session_state:
        st.session_state.target_ph_value = 7.0
    if "ph_status" not in st.session_state:
        st.session_state.ph_status = "🟠" 

    if "rpm_history_df" not in st.session_state:
        st.session_state.rpm_history_df = pd.DataFrame(columns=["time", "rpm"])
    if "temp_history_df" not in st.session_state:
        st.session_state.temp_history_df = pd.DataFrame(columns=["time", "temp"])
    if "ph_history_df" not in st.session_state:
        st.session_state.ph_history_df = pd.DataFrame(columns=["time", "ph"])

    if "mqtt_last_updated" not in st.session_state:
        st.session_state.mqtt_last_updated = 0


@st.cache_resource
def start_mqtt_client():
    def on_message(client, userdata, msg):
        try:
            payload = msg.payload.decode()
            data = json.loads(payload) 
            
            print(f"on_message: {data}")
            incoming_mqtt_queue.put(data)

            
        except json.JSONDecodeError as json_e:
            print(f"ERROR: JSON Decode Failed! Payload: {msg.payload.decode()} Error: {json_e}")
    
        except Exception as e:
            print(f"ERROR (Thread Conflict): State update failed. {e}")

    client = mqtt.Client()
    
    client.tls_set(cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLS)
    client.username_pw_set(
        st.secrets["hivemq"]["username"], 
        st.secrets["hivemq"]["password"]
    )
    
    client.on_message = on_message
    
    try:
        client.connect(st.secrets["hivemq"]["broker"], st.secrets["hivemq"]["port"])
        client.subscribe(st.secrets["hivemq"]["topic_subscribe"])
        
        client.loop_start() 
        print("✅ MQTT Connected")
    except Exception as e:
        print(f"❌ Connection Failed: {e}")
        
    return client

def get_mqtt_status(client):
    if not client.is_connected():
        return 0, "🔴Connection Fail"
    
    last_update = st.session_state.get("mqtt_last_updated", 0)

    if last_update == 0:
        return 1, "🟠Waiting For Data"
    
    if (time.time()-last_update)>10:
        return 2, "🟡Transfer Interrupted"
    
    return 3, "🟢Working Good"
    
#grab data from the queue
def safe_update_session_state():
    while not incoming_mqtt_queue.empty():
        try:
            data = incoming_mqtt_queue.get_nowait()

            #update the chart/data
            if "rpm" in data:
                #update data
                st.session_state.current_stirring_rpm = int (data["rpm"])

                #update status
                st.session_state.stirring_status = "🟢"

                #update chart
                new_rpm_value = data["rpm"]
                new_row = pd.DataFrame({"time": [time.time()], "rpm": [new_rpm_value]})
                
                st.session_state.rpm_history_df = pd.concat(
                    [st.session_state.rpm_history_df, new_row], 
                    ignore_index=True
                )
                
                if len(st.session_state.rpm_history_df) > 20:
                    st.session_state.rpm_history_df = st.session_state.rpm_history_df.tail(20).reset_index(drop=True)
            else:
                st.session_state.stirring_status = "🔴"

            if "temp" in data:
                #update data
                st.session_state.current_heating_temp = float(data["temp"])

                #update status
                st.session_state.heating_status = "🟢"

                #update chart
                new_temp_value = data["temp"]
                new_row = pd.DataFrame({"time": [time.time()], "temp": [new_temp_value]})
                
                st.session_state.temp_history_df = pd.concat(
                    [st.session_state.temp_history_df, new_row], 
                    ignore_index=True
                )
                
                if len(st.session_state.temp_history_df) > 20:
                    st.session_state.temp_history_df = st.session_state.temp_history_df.tail(20).reset_index(drop=True)
            else:
                st.session_state.heating_status = "🔴"

            if "ph" in data:
                #update data
                st.session_state.current_ph_value = float(data["ph"])

                #update status
                st.session_state.ph_status = "🟢"

                #update chart
                new_ph_value = data["ph"]
                new_row = pd.DataFrame({"time": [time.time()], "ph": [new_ph_value]})
                
                st.session_state.ph_history_df = pd.concat(
                    [st.session_state.ph_history_df, new_row], 
                    ignore_index=True
                )
                
                if len(st.session_state.ph_history_df) > 20:
                    st.session_state.ph_history_df = st.session_state.ph_history_df.tail(20).reset_index(drop=True)
            else:
                st.session_state.ph_status = "🔴"

            #update time
            st.session_state.mqtt_last_updated = time.time()
            #print(f"{st.session_state.mqtt_last_updated}safe_update_session_state")
            
        except queue.Empty:
            break
        except Exception as e:
            print(f"Update error: {e}")

    