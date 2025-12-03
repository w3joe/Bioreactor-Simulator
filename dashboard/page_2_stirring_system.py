import streamlit as st
import numpy as np
import pandas as pd
import time
import mqtt_manager as mm
import json
import plotly.graph_objects as go
from mqtt_manager import initialize_session_state, start_mqtt_client, safe_update_session_state

mm.initialize_session_state()
client = mm.start_mqtt_client()

st.title("⚙️ Stirring System Dashboard")
st.markdown("Control and monitor the bioreactor stirring motor in real-time")
st.sidebar.markdown("# Stirring System")
st.divider()

# =========================================================================
# System Status Display
# =========================================================================
@st.fragment(run_every=1)
def display_dashboard():
    s1, s2, s3 = st.columns([1.2, 1.2, 1])

    with s1:
        with st.container(border=True):
            st.metric(
                label="Current Speed",
                value=f"{st.session_state.current_stirring_rpm} RPM",
                delta=f"{st.session_state.current_stirring_rpm - st.session_state.target_stirring_rpm:.0f} RPM" if st.session_state.target_stirring_rpm > 0 else None
            )

    with s2:
        with st.container(border=True):
            st.metric(
                label="Target Speed",
                value=f"{st.session_state.target_stirring_rpm} RPM"
            )

    with s3:
        with st.container(border=True):
            st.markdown("**System Status**")
            st.markdown(f"# {st.session_state.stirring_status}")

display_dashboard()
st.divider()

# =========================================================================
# Control Panel
# =========================================================================
st.markdown("### 🎛️ Control Panel")
st.markdown("Adjust the target stirring speed and send commands to the bioreactor")

def display_control_panel():
    with st.container(border=True):
        col_slider, col_btn = st.columns([4, 1])
        with col_slider:
            st.slider(
                "Set Target RPM",
                min_value=0,
                max_value=1000,
                value=st.session_state.target_stirring_rpm,
                step=50,
                key="stirring_slider_input",
                help="Adjust the target stirring speed (0-1000 RPM)"
            )

        with col_btn:
            st.write("##")

            if st.button("🚀 SEND", key="send_stirring_btn", width='stretch', type="primary"):
                new_target = st.session_state.stirring_slider_input
                st.session_state.target_stirring_rpm = new_target

                payload = json.dumps({"target_rpm": new_target})
                client.publish(st.secrets["hivemq"]["topic_publish"], payload)
                st.toast(f"✅ RPM command sent: {new_target} RPM", icon="🚀")

display_control_panel()
st.divider()

# =========================================================================
# Real-Time Chart with Dynamic Animation
# =========================================================================
@st.fragment(run_every=0.5)
def display_line_chart():
    st.markdown("### 📊 Real-time RPM Trend")
    st.markdown("*Displaying the last 20 data points with live animation*")

    if "rpm_history_df" in st.session_state and not st.session_state.rpm_history_df.empty:
        df = st.session_state.rpm_history_df.copy()

        if len(df) > 0:
            df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')

            with st.container(border=True):
                # Create Plotly chart with thicker lines
                fig = go.Figure()
                fig.add_trace(go.Scatter(
                    x=df['Time'],
                    y=df['rpm'],
                    mode='lines',
                    line=dict(color='#FF4B4B', width=3),
                    name='RPM'
                ))
                fig.update_layout(
                    height=400,
                    margin=dict(l=0, r=0, t=0, b=0),
                    xaxis_title='Time',
                    yaxis_title='RPM',
                    hovermode='x unified',
                    plot_bgcolor='rgba(0,0,0,0)',
                    paper_bgcolor='rgba(0,0,0,0)',
                    font=dict(color='white')
                )
                st.plotly_chart(fig, width='stretch')

                # Statistics
                col1, col2, col3, col4 = st.columns(4)
                with col1:
                    st.metric("Latest", f"{df['rpm'].iloc[-1]:.1f} RPM")
                with col2:
                    st.metric("Average", f"{df['rpm'].mean():.1f} RPM")
                with col3:
                    st.metric("Max", f"{df['rpm'].max():.1f} RPM")
                with col4:
                    st.metric("Min", f"{df['rpm'].min():.1f} RPM")
    else:
        # Display placeholder chart when no data
        placeholder_df = pd.DataFrame({
            'Time': [f"{i:02d}:00" for i in range(10)],
            'rpm': [0] * 10
        })

        with st.container(border=True):
            fig = go.Figure()
            fig.add_trace(go.Scatter(
                x=placeholder_df['Time'],
                y=placeholder_df['rpm'],
                mode='lines',
                line=dict(color='#FF4B4B', width=3),
                name='RPM'
            ))
            fig.update_layout(
                height=400,
                margin=dict(l=0, r=0, t=0, b=0),
                xaxis_title='Time',
                yaxis_title='RPM',
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white')
            )
            st.plotly_chart(fig, width='stretch')
            st.info("⏳ Waiting for RPM data stream... Make sure the ESP32 TTGO is connected and publishing data.")

display_line_chart()

# =========================================================================
# System Information
# =========================================================================
with st.expander("ℹ️ Stirring System Information"):
    st.markdown("""
    ### Bang-Bang Controller Configuration

    - **Control Range**: 0 - 1000 RPM
    - **PWM Resolution**: 10-bit (0-1023)
    - **PWM Frequency**: 5 kHz
    - **Control Loop Rate**: 10 ms (100 Hz)
    - **Hall Sensor**: 70 pulses per revolution
    - **Controller Type**: Bang-bang with hysteresis

    ### How It Works:
    1. Hall sensor detects magnetic pulses from motor
    2. Frequency converted to RPM (60 × freq / 70)
    3. Bang-bang controller outputs either 0 (OFF) or 1023 (FULL SPEED)
    4. Hysteresis prevents oscillation around setpoint

    ### Control Commands:
    Send commands via MQTT topic: `ucl/ec2/group5/ttgo/command`

    Example: `{"target_rpm": 500}`
    """)
