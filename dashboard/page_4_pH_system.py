import streamlit as st
import numpy as np
import pandas as pd
import time
import mqtt_manager as mm
import json

mm.initialize_session_state()
client = mm.start_mqtt_client()

st.title("pH System Dashboard", help="this dashboard is for our pH system")
st.sidebar.markdown("# pH System")

@st.fragment(run_every=1)
def display_dashboard():
    s1, s2, s3 = st.columns([1,1,1.1])

    with s1:
        with st.container(border=True):
            st.header(f" {st.session_state.current_ph_value} ")
            st.write("Current pH")

    with s2:
        with st.container(border=True):
            st.header(f" {st.session_state.target_ph_value} ")
            st.write("Target ph")

    with s3:
        with st.container(border=True):
            st.header(f" {st.session_state.ph_status} ")
            st.write("Status")

def display_control_panel():
    with st.container(border=True):
        col_slider, col_btn = st.columns([4,1])
        with col_slider:
          st.slider(
               "Set pH",
               min_value=1.0, 
               max_value=13.0, 
               value=st.session_state.target_ph_value, 
               step=0.5,
               key="pH_slider_input"
          ) 
        
        with col_btn:
            st.write("##") 
            
            if st.button("🚀 SEND", key="send_pH_btn", width="stretch", type="primary"):
               new_target = st.session_state.pH_slider_input 
               st.session_state.target_ph_value = new_target
               
               payload = json.dumps({"target_pH": new_target}) 
               client.publish(st.secrets["hivemq"]["topic_publish"], payload)
               st.toast(f"pH command sent: {new_target}", icon="🚀")

@st.fragment(run_every=1)
def display_line_chart():
    st.subheader("📊 Real-time pH Trend")
    st.caption("(Last 20 Potins)")
    
    if "ph_history_df" in st.session_state:
        df = st.session_state.ph_history_df.copy()
    else:
        df = pd.DataFrame()
    
    if not df.empty:
        df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')
        
        st.line_chart(
            df, 
            x='Time', 
            y='ph',
            color="#8A2BE2", 
            width="stretch"
        )
    else:
        st.info("Waiting for pH data stream...")
    

display_dashboard()
st.divider()
st.header("Control Panel")
display_control_panel()
st.divider()
display_line_chart()