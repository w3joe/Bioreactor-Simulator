import streamlit as st
import numpy as np
import pandas as pd
import time
import mqtt_manager as mm
import json
from mqtt_manager import initialize_session_state, start_mqtt_client, safe_update_session_state

mm.initialize_session_state()
client = mm.start_mqtt_client()


st.title("Stirring System Dashboard", help="this dashboard is for our stirring system")
st.sidebar.markdown("# Stirring System")

@st.fragment(run_every=1)
def display_dashboard():
    #print(f"display_dashboard_stirring: {time.time()}")
    s1, s2, s3 = st.columns([1.2,1.2,1])

    with s1:
        with st.container(border=True):
            st.header(f" {st.session_state.current_stirring_rpm} RPM")
            st.write("Current Speed")

    with s2:
        with st.container(border=True):
            st.header(f" {st.session_state.target_stirring_rpm} RPM")
            st.write("Target Speed")

    with s3:
        with st.container(border=True):
            st.header(f" {st.session_state.stirring_status} ")
            st.write("Status")

def display_control_panel():
    with st.container(border=True):
        col_slider, col_btn = st.columns([4,1])
        with col_slider:
          st.slider(
               "Set RPM",
               min_value=0, 
               max_value=1000, 
               value=st.session_state.target_stirring_rpm, 
               step=50,
               key="stirring_slider_input"
          ) 
        
        with col_btn:
            st.write("##") 
            
            if st.button("🚀 SEND", key="send_stirring_btn", width="stretch", type="primary"):
               new_target = st.session_state.stirring_slider_input
               st.session_state.target_stirring_rpm = new_target
               
               payload = json.dumps({"target_rpm": new_target}) 
               client.publish(st.secrets["hivemq"]["topic_publish"], payload)
               st.toast(f"pH command sent: {new_target}", icon="🚀")

@st.fragment(run_every=1)
def display_line_chart():
    st.header("📊 Real-time RPM Trend")
    st.subheader("(Last 20 Points)")

    if "rpm_history_df" in st.session_state:
        df = st.session_state.rpm_history_df.copy()
    else:
        df = pd.DataFrame()
    
    if not df.empty:
        df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')
        
        st.line_chart(
            df, 
            x='Time', 
            y='rpm',
            color="#FF4B4B", 
            width="stretch"
        )
    else:
        st.info("Waiting for data stream...")
    

display_dashboard()
st.divider()
st.header("Control Panel")
display_control_panel()
st.divider()
display_line_chart()