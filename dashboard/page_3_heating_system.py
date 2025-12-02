import streamlit as st
import numpy as np
import pandas as pd
import time
import mqtt_manager as mm
import json

mm.initialize_session_state()
client = mm.start_mqtt_client()


st.title("Heating System Dashboard", help="this dashboard is for our heating system")
st.sidebar.markdown("# Heating System")

@st.fragment(run_every=1)
def display_dashboard():
    s1, s2, s3 = st.columns([1,1,1.1])

    with s1:
        with st.container(border=True):
            st.header(f" {st.session_state.current_heating_temp} ")
            st.write("Current Temp")

    with s2:
        with st.container(border=True):
            st.header(f" {st.session_state.target_heating_temp} ")
            st.write("Target Temp")

    with s3:
        with st.container(border=True):
            st.header(f" {st.session_state.heating_status} ")
            st.write("Status")

def display_control_panel():
    with st.container(border=True):
        col_slider, col_btn = st.columns([4,1])
        with col_slider:
          st.slider(
               "Set Target Temp (°C)",
               min_value=0.0, 
               max_value=50.0, 
               value=st.session_state.target_heating_temp, 
               step=0.5,
               key="temp_slider_input"
          ) 
        
        with col_btn:
            st.write("##") 
            
            if st.button("🚀 SEND", key="send_temp_btn", width="stretch", type="primary"):
               new_target = st.session_state.temp_slider_input 
               st.session_state.target_heating_temp = new_target
               
               payload = json.dumps({"target_temp": new_target}) 
               client.publish(st.secrets["hivemq"]["topic_publish"], payload)
               st.toast(f"Temp command sent: {new_target}", icon="🚀")

@st.fragment(run_every=1)
def display_line_chart():
    st.header("📊 Real-time Temperature Trend")
    st.subheader("(Last 20 Points)")

    if "temp_history_df" in st.session_state:
        df = st.session_state.temp_history_df.copy()
    else:
        df = pd.DataFrame()
    
    if not df.empty:
        df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')
        
        st.line_chart(
            df, 
            x='Time', 
            y='temp',
            color="#FFA500",  
            width="stretch"
        )
    else:
        st.info("Waiting for temperature data stream...")
    

display_dashboard()
st.divider()
st.header("Control Panel")
display_control_panel()
st.divider()
display_line_chart()