import streamlit as st
import numpy as np
import pandas as pd
import time
import mqtt_manager as mm
from datetime import datetime
import plotly.graph_objects as go

mm.initialize_session_state()
client = mm.start_mqtt_client()

# Page configuration
st.title("🧬 Bioreactor Real-Time Monitoring Dashboard")
st.markdown("### Welcome to UCL Engineering Challenges - Group 5")
st.markdown("Monitor and control your bioreactor system in real-time through MQTT communication")
st.divider()

# =========================================================================
# System Status Cards
# =========================================================================
@st.fragment(run_every=1)
def display_system_status():
    st.markdown("### 📊 System Overview")

    col1, col2, col3 = st.columns(3)

    with col1:
        with st.container(border=True):
            st.markdown("#### ⚙️ Stirring System")
            st.metric(
                label="Current Speed",
                value=f"{st.session_state.current_stirring_rpm} RPM",
                delta=f"Target: {st.session_state.target_stirring_rpm} RPM" if st.session_state.target_stirring_rpm > 0 else None
            )
            st.markdown(f"**Status:** {st.session_state.stirring_status}")

    with col2:
        with st.container(border=True):
            st.markdown("#### 🔥 Heating System")
            st.metric(
                label="Current Temperature",
                value=f"{st.session_state.current_heating_temp:.1f} °C",
                delta=f"Target: {st.session_state.target_heating_temp:.1f} °C" if st.session_state.target_heating_temp > 0 else None
            )
            st.markdown(f"**Status:** {st.session_state.heating_status}")

    with col3:
        with st.container(border=True):
            st.markdown("#### 🧪 pH System")
            st.metric(
                label="Current pH",
                value=f"{st.session_state.current_ph_value:.2f}",
                delta=f"Target: {st.session_state.target_ph_value:.2f}" if st.session_state.target_ph_value > 0 else None
            )
            st.markdown(f"**Status:** {st.session_state.ph_status}")

display_system_status()
st.divider()

# =========================================================================
# Real-Time Charts Section
# =========================================================================
st.markdown("### 📈 Real-Time Data Trends")
st.markdown("*Displaying the last 20 data points for each parameter*")

# RPM Chart with dynamic animation
@st.fragment(run_every=0.5)
def display_rpm_chart():
    with st.container(border=True):
        st.markdown("#### ⚙️ Stirring Motor Speed (RPM)")

        if "rpm_history_df" in st.session_state and not st.session_state.rpm_history_df.empty:
            df = st.session_state.rpm_history_df.copy()

            # Convert time to relative seconds for smoother animation
            if len(df) > 0:
                df['seconds'] = df['time'] - df['time'].iloc[0]
                df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')

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
                    height=300,
                    margin=dict(l=0, r=0, t=0, b=0),
                    xaxis_title='Time',
                    yaxis_title='RPM',
                    hovermode='x unified',
                    plot_bgcolor='rgba(0,0,0,0)',
                    paper_bgcolor='rgba(0,0,0,0)',
                    font=dict(color='white')
                )
                st.plotly_chart(fig, width='stretch')

                # Show latest value with timestamp
                latest_rpm = df['rpm'].iloc[-1]
                latest_time = df['Time'].iloc[-1]
                col1, col2, col3 = st.columns(3)
                with col1:
                    st.metric("Latest", f"{latest_rpm:.1f} RPM")
                with col2:
                    st.metric("Average", f"{df['rpm'].mean():.1f} RPM")
                with col3:
                    st.caption(f"Updated: {latest_time}")
        else:
            # Display placeholder chart when no data
            placeholder_df = pd.DataFrame({
                'Time': [f"{i:02d}:00" for i in range(10)],
                'rpm': [0] * 10
            })

            fig = go.Figure()
            fig.add_trace(go.Scatter(
                x=placeholder_df['Time'],
                y=placeholder_df['rpm'],
                mode='lines',
                line=dict(color='#FF4B4B', width=3),
                name='RPM'
            ))
            fig.update_layout(
                height=300,
                margin=dict(l=0, r=0, t=0, b=0),
                xaxis_title='Time',
                yaxis_title='RPM',
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white')
            )
            st.plotly_chart(fig, width='stretch')
            st.info("⏳ Waiting for RPM data stream...")

# Temperature Chart with dynamic animation
@st.fragment(run_every=0.5)
def display_temp_chart():
    with st.container(border=True):
        st.markdown("#### 🔥 Temperature (°C)")

        if "temp_history_df" in st.session_state and not st.session_state.temp_history_df.empty:
            df = st.session_state.temp_history_df.copy()

            # Convert time to relative seconds for smoother animation
            if len(df) > 0:
                df['seconds'] = df['time'] - df['time'].iloc[0]
                df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')

                # Create Plotly chart with thicker lines
                fig = go.Figure()
                fig.add_trace(go.Scatter(
                    x=df['Time'],
                    y=df['temp'],
                    mode='lines',
                    line=dict(color='#FFA500', width=3),
                    name='Temperature'
                ))
                fig.update_layout(
                    height=300,
                    margin=dict(l=0, r=0, t=0, b=0),
                    xaxis_title='Time',
                    yaxis_title='Temperature (°C)',
                    hovermode='x unified',
                    plot_bgcolor='rgba(0,0,0,0)',
                    paper_bgcolor='rgba(0,0,0,0)',
                    font=dict(color='white')
                )
                st.plotly_chart(fig,  width='stretch')

                # Show latest value with timestamp
                latest_temp = df['temp'].iloc[-1]
                latest_time = df['Time'].iloc[-1]
                col1, col2, col3 = st.columns(3)
                with col1:
                    st.metric("Latest", f"{latest_temp:.1f} °C")
                with col2:
                    st.metric("Average", f"{df['temp'].mean():.1f} °C")
                with col3:
                    st.caption(f"Updated: {latest_time}")
        else:
            # Display placeholder chart when no data
            placeholder_df = pd.DataFrame({
                'Time': [f"{i:02d}:00" for i in range(10)],
                'temp': [20.0] * 10
            })

            fig = go.Figure()
            fig.add_trace(go.Scatter(
                x=placeholder_df['Time'],
                y=placeholder_df['temp'],
                mode='lines',
                line=dict(color='#FFA500', width=3),
                name='Temperature'
            ))
            fig.update_layout(
                height=300,
                margin=dict(l=0, r=0, t=0, b=0),
                xaxis_title='Time',
                yaxis_title='Temperature (°C)',
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white')
            )
            st.plotly_chart(fig,  width='stretch')
            st.info("⏳ Waiting for temperature data stream...")

# pH Chart with dynamic animation
@st.fragment(run_every=0.5)
def display_ph_chart():
    with st.container(border=True):
        st.markdown("#### 🧪 pH Level")

        if "ph_history_df" in st.session_state and not st.session_state.ph_history_df.empty:
            df = st.session_state.ph_history_df.copy()

            # Convert time to relative seconds for smoother animation
            if len(df) > 0:
                df['seconds'] = df['time'] - df['time'].iloc[0]
                df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')

                # Create Plotly chart with thicker lines
                fig = go.Figure()
                fig.add_trace(go.Scatter(
                    x=df['Time'],
                    y=df['ph'],
                    mode='lines',
                    line=dict(color='#8A2BE2', width=3),
                    name='pH'
                ))
                fig.update_layout(
                    height=300,
                    margin=dict(l=0, r=0, t=0, b=0),
                    xaxis_title='Time',
                    yaxis_title='pH',
                    hovermode='x unified',
                    plot_bgcolor='rgba(0,0,0,0)',
                    paper_bgcolor='rgba(0,0,0,0)',
                    font=dict(color='white')
                )
                st.plotly_chart(fig,  width='stretch')

                # Show latest value with timestamp
                latest_ph = df['ph'].iloc[-1]
                latest_time = df['Time'].iloc[-1]
                col1, col2, col3 = st.columns(3)
                with col1:
                    st.metric("Latest", f"{latest_ph:.2f}")
                with col2:
                    st.metric("Average", f"{df['ph'].mean():.2f}")
                with col3:
                    st.caption(f"Updated: {latest_time}")
        else:
            # Display placeholder chart when no data
            placeholder_df = pd.DataFrame({
                'Time': [f"{i:02d}:00" for i in range(10)],
                'ph': [7.0] * 10
            })

            fig = go.Figure()
            fig.add_trace(go.Scatter(
                x=placeholder_df['Time'],
                y=placeholder_df['ph'],
                mode='lines',
                line=dict(color='#8A2BE2', width=3),
                name='pH'
            ))
            fig.update_layout(
                height=300,
                margin=dict(l=0, r=0, t=0, b=0),
                xaxis_title='Time',
                yaxis_title='pH',
                plot_bgcolor='rgba(0,0,0,0)',
                paper_bgcolor='rgba(0,0,0,0)',
                font=dict(color='white')
            )
            st.plotly_chart(fig,  width='stretch')
            st.info("⏳ Waiting for pH data stream...")

# Display all charts
display_rpm_chart()
display_temp_chart()
display_ph_chart()

st.divider()

# =========================================================================
# System Information
# =========================================================================
with st.expander("ℹ️ System Information", expanded=False):
    st.markdown("""
    ### About This Dashboard

    **Bioreactor Control System** - UCL Engineering Challenges 2024/2025

    This dashboard provides real-time monitoring and control of a bioreactor system using:
    - **ESP32 DevKit**: Main controller board managing sensors and actuators
    - **ESP32 TTGO**: Display and MQTT gateway board
    - **HiveMQ Cloud**: MQTT broker for bidirectional communication
    - **Streamlit**: Real-time web dashboard

    #### Controlled Parameters:
    - 🔄 **Stirring Speed**: 0-1000 RPM using bang-bang control
    - 🌡️ **Temperature**: 20-45°C with 39% power limit
    - 🧪 **pH Level**: 4.0-10.0 with dual pump control

    #### Data Flow:
    1. Sensors → ESP32 DevKit (control loops at 10ms)
    2. DevKit → TTGO via UART (100ms update rate)
    3. TTGO → MQTT Broker (5 second publish interval)
    4. Dashboard subscribes to MQTT topic for real-time updates

    #### MQTT Topics:
    - **Publish**: `ucl/ec2/group5/ttgo/data`
    - **Subscribe**: `ucl/ec2/group5/ttgo/command`

    **Group 5 Members**: [Add your names here]
    """)

# Add navigation hint
st.info("💡 **Tip**: Use the sidebar to navigate to individual system control pages for detailed controls and settings.")
