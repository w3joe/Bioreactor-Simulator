import streamlit as st
import numpy as np
import pandas as pd
import time
import mqtt_manager as mm
import json

mm.initialize_session_state()
client = mm.start_mqtt_client()

st.title("🧪 pH System Dashboard")
st.markdown("Control and monitor the bioreactor pH level in real-time")
st.sidebar.markdown("# pH System")
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
                label="Current pH",
                value=f"{st.session_state.current_ph_value:.2f}",
                delta=f"{st.session_state.current_ph_value - st.session_state.target_ph_value:.2f}" if st.session_state.target_ph_value > 0 else None
            )

    with s2:
        with st.container(border=True):
            st.metric(
                label="Target pH",
                value=f"{st.session_state.target_ph_value:.2f}"
            )

    with s3:
        with st.container(border=True):
            st.markdown("**System Status**")
            st.markdown(f"# {st.session_state.ph_status}")

display_dashboard()
st.divider()

# =========================================================================
# Control Panel
# =========================================================================
st.markdown("### 🎛️ Control Panel")
st.markdown("Adjust the target pH level and send commands to the bioreactor")

def display_control_panel():
    with st.container(border=True):
        col_slider, col_btn = st.columns([4, 1])
        with col_slider:
            st.slider(
                "Set Target pH",
                min_value=4.0,
                max_value=10.0,
                value=st.session_state.target_ph_value,
                step=0.5,
                key="pH_slider_input",
                help="Adjust the target pH level (4.0-10.0)"
            )

        with col_btn:
            st.write("##")

            if st.button("🚀 SEND", key="send_pH_btn", width='stretch', type="primary"):
                new_target = st.session_state.pH_slider_input
                st.session_state.target_ph_value = new_target

                payload = json.dumps({"target_pH": new_target})
                client.publish(st.secrets["hivemq"]["topic_publish"], payload)
                st.toast(f"✅ pH command sent: {new_target:.2f}", icon="🚀")

display_control_panel()
st.divider()

# =========================================================================
# Real-Time Chart with Dynamic Animation
# =========================================================================
@st.fragment(run_every=0.5)
def display_line_chart():
    st.markdown("### 📊 Real-time pH Trend")
    st.markdown("*Displaying the last 20 data points with live animation*")

    if "ph_history_df" in st.session_state and not st.session_state.ph_history_df.empty:
        df = st.session_state.ph_history_df.copy()

        if len(df) > 0:
            df['Time'] = pd.to_datetime(df['time'], unit='s').dt.strftime('%H:%M:%S')

            with st.container(border=True):
                # Create chart with Time as index for smooth animation
                chart_df = df.set_index('Time')[['ph']]

                st.line_chart(
                    chart_df,
                    width='stretch',
                    height=400
                )

                # Statistics
                col1, col2, col3, col4 = st.columns(4)
                with col1:
                    st.metric("Latest", f"{df['ph'].iloc[-1]:.2f}")
                with col2:
                    st.metric("Average", f"{df['ph'].mean():.2f}")
                with col3:
                    st.metric("Max", f"{df['ph'].max():.2f}")
                with col4:
                    st.metric("Min", f"{df['ph'].min():.2f}")
    else:
        # Display placeholder chart when no data
        placeholder_df = pd.DataFrame({
            'Time': [f"{i:02d}:00" for i in range(10)],
            'ph': [7.0] * 10
        }).set_index('Time')

        with st.container(border=True):
            st.line_chart(
                placeholder_df,
                width='stretch',
                height=400
            )
            st.info("⏳ Waiting for pH data stream... Make sure the ESP32 TTGO is connected and publishing data.")

display_line_chart()

# =========================================================================
# System Information
# =========================================================================
with st.expander("ℹ️ pH System Information"):
    st.markdown("""
    ### Dual Pump Control System

    - **Control Range**: 4.0 - 10.0 pH
    - **Deadband**: ±0.2 pH
    - **Pump Runtime**: 3 seconds per dose
    - **Pump Cooldown**: 5 seconds between doses
    - **Sensor**: Analog pH sensor with moving average filter
    - **Sampling Rate**: 20 ms
    - **Filter Length**: 40 samples (800 ms window)

    ### Pump Configuration:
    - **Pump A (Motor A - GPIO25)**: Base pump (raises pH)
    - **Pump B (Motor B - GPIO26)**: Acid pump (lowers pH)
    - **PWM Range**: 0-255 (8-bit)
    - **PWM Ramp**: Smooth ramping for motor protection
    - **Startup**: 42-second priming sequence for each pump

    ### pH Sensor Calibration:
    - **Equation**: pH = 0.0064 × ADC + 3.0644
    - **ADC Range**: 0-4095 (12-bit)
    - **Attenuation**: 11dB (supports 0-3.3V)

    ### How It Works:
    1. pH sensor readings averaged over 40 samples
    2. Calibration equation converts ADC to pH value
    3. Control logic with deadband prevents oscillation:
       - If pH < (target - 0.2): Pump A doses base for 3 seconds
       - If pH > (target + 0.2): Pump B doses acid for 3 seconds
       - Within band: no action
    4. 5-second cooldown prevents overdosing
    5. PWM ramp ensures smooth motor startup/shutdown

    ### Startup Sequence:
    1. **Phase 0** (0-42s): Prime Pump A
    2. **Phase 1** (42-84s): Prime Pump B
    3. **Phase 2** (84s+): Normal pH control operation

    ### Control Commands:
    Send commands via MQTT topic: `ucl/ec2/group5/ttgo/command`

    Example: `{"target_pH": 7.0}`
    """)
