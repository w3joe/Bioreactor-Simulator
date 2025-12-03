import streamlit as st
import numpy as np
import pandas as pd
import time
import mqtt_manager as mm
import json

mm.initialize_session_state()
client = mm.start_mqtt_client()

st.title("🔥 Heating System Dashboard")
st.markdown("Control and monitor the bioreactor temperature in real-time")
st.sidebar.markdown("# Heating System")
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
                label="Current Temperature",
                value=f"{st.session_state.current_heating_temp:.1f} °C",
                delta=f"{st.session_state.current_heating_temp - st.session_state.target_heating_temp:.1f} °C" if st.session_state.target_heating_temp > 0 else None
            )

    with s2:
        with st.container(border=True):
            st.metric(
                label="Target Temperature",
                value=f"{st.session_state.target_heating_temp:.1f} °C"
            )

    with s3:
        with st.container(border=True):
            st.markdown("**System Status**")
            st.markdown(f"# {st.session_state.heating_status}")

display_dashboard()
st.divider()

# =========================================================================
# Control Panel
# =========================================================================
st.markdown("### 🎛️ Control Panel")
st.markdown("Adjust the target temperature and send commands to the bioreactor")

def display_control_panel():
    with st.container(border=True):
        col_slider, col_btn = st.columns([4, 1])
        with col_slider:
            st.slider(
                "Set Target Temperature (°C)",
                min_value=20.0,
                max_value=45.0,
                value=st.session_state.target_heating_temp,
                step=0.5,
                key="temp_slider_input",
                help="Adjust the target temperature (20-45°C)"
            )

        with col_btn:
            st.write("##")

            if st.button("🚀 SEND", key="send_temp_btn", width='stretch', type="primary"):
                new_target = st.session_state.temp_slider_input
                st.session_state.target_heating_temp = new_target

                payload = json.dumps({"target_temp": new_target})
                client.publish(st.secrets["hivemq"]["topic_publish"], payload)
                st.toast(f"✅ Temperature command sent: {new_target:.1f} °C", icon="🚀")

display_control_panel()
st.divider()

# =========================================================================
# Real-Time Chart with Dynamic Animation
# =========================================================================
@st.fragment(run_every=0.5)
def display_line_chart():
    st.markdown("### 📊 Real-time Temperature Trend")
    st.markdown("*Displaying the last 20 data points with live animation*")

    if "temp_history_df" in st.session_state and not st.session_state.temp_history_df.empty:
        df = st.session_state.temp_history_df.copy()

        if len(df) > 0:
            df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')

            with st.container(border=True):
                # Create chart with Time as index for smooth animation
                chart_df = df.set_index('Time')[['temp']]

                st.line_chart(
                    chart_df,
                    width='stretch',
                    height=400
                )

                # Statistics
                col1, col2, col3, col4 = st.columns(4)
                with col1:
                    st.metric("Latest", f"{df['temp'].iloc[-1]:.1f} °C")
                with col2:
                    st.metric("Average", f"{df['temp'].mean():.1f} °C")
                with col3:
                    st.metric("Max", f"{df['temp'].max():.1f} °C")
                with col4:
                    st.metric("Min", f"{df['temp'].min():.1f} °C")
    else:
        # Display placeholder chart when no data
        placeholder_df = pd.DataFrame({
            'Time': [f"{i:02d}:00" for i in range(10)],
            'temp': [20.0] * 10
        }).set_index('Time')

        with st.container(border=True):
            st.line_chart(
                placeholder_df,
                width='stretch',
                height=400
            )
            st.info("⏳ Waiting for temperature data stream... Make sure the ESP32 TTGO is connected and publishing data.")

display_line_chart()

# =========================================================================
# System Information
# =========================================================================
with st.expander("ℹ️ Heating System Information"):
    st.markdown("""
    ### Bang-Bang Controller Configuration

    - **Control Range**: 20 - 45 °C
    - **PWM Resolution**: 10-bit (0-1023)
    - **Power Limit**: 39% (PWM value: 550)
    - **Control Loop Rate**: 10 ms (100 Hz)
    - **Hysteresis**: ±0.5 °C
    - **Sensor**: Thermistor with Steinhart-Hart equation
    - **Controller Type**: Bang-bang with hysteresis

    ### Thermistor Specifications:
    - **Series Resistor**: 10kΩ
    - **ADC Resolution**: 12-bit (0-4095)
    - **Reference Voltage**: 3.3V
    - **Steinhart-Hart Coefficients**:
      - A = 1.009249522e-03
      - B = 2.378405444e-04
      - C = 2.019202697e-07

    ### How It Works:
    1. Thermistor resistance measured via voltage divider
    2. Steinhart-Hart equation converts resistance to temperature
    3. Bang-bang controller with hysteresis prevents oscillation
    4. If temp < (target - 0.5°C): heater ON at 39% power
    5. If temp > (target + 0.5°C): heater OFF
    6. Within band: maintain previous state

    ### Control Commands:
    Send commands via MQTT topic: `ucl/ec2/group5/ttgo/command`

    Example: `{"target_temp": 35.0}`
    """)
