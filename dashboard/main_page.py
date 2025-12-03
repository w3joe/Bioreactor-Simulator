import streamlit as st
import numpy as np
import pandas as pd
import time

st.title("🏆 Welcome to the Bioreactor Control System")
st.markdown("### UCL Engineering Challenges 2024/2025 - Group 5")

st.divider()

# Hero section
col1, col2 = st.columns([2, 1])

with col1:
    st.markdown("""
    ## About This Project

    This advanced bioreactor control system demonstrates real-time monitoring and control
    of critical bioreactor parameters using embedded systems and IoT technologies.

    ### Key Features:
    - 🔄 **Real-time Monitoring**: Live data streaming via MQTT
    - 🎛️ **Remote Control**: Adjust setpoints from anywhere
    - 📊 **Visual Analytics**: Interactive charts and metrics
    - ⚡ **Low Latency**: Sub-second response times
    - 🔒 **Secure Communication**: TLS-encrypted MQTT over HiveMQ Cloud

    ### System Architecture:
    - **ESP32 DevKit**: Control loops running at 100 Hz
    - **ESP32 TTGO**: Local display + MQTT gateway
    - **Streamlit Dashboard**: Web-based monitoring interface
    - **HiveMQ Cloud**: Enterprise MQTT broker
    """)

with col2:
    with st.container(border=True):
        st.markdown("### 🚀 Quick Start")
        st.markdown("""
        1. Ensure ESP32 TTGO is powered on
        2. Check WiFi connection (eduroam)
        3. Verify MQTT status in sidebar
        4. Navigate to system pages:
           - ⚙️ Stirring System
           - 🔥 Heating System
           - 🧪 pH System
        5. Monitor real-time data
        6. Adjust setpoints as needed
        """)

        st.divider()

        st.markdown("### 📡 System Status")
        st.markdown("Check the sidebar for:")
        st.markdown("- 🔗 MQTT Connection Status")
        st.markdown("- 🌐 Network Latency")
        st.markdown("- 📍 Public IP Address")

st.divider()

# Three column feature showcase
st.markdown("## 🔬 Controlled Parameters")

col1, col2, col3 = st.columns(3)

with col1:
    with st.container(border=True):
        st.markdown("### ⚙️ Stirring Speed")
        st.markdown("""
        **Range**: 0 - 1000 RPM

        Precise motor speed control using bang-bang control
        with hysteresis. Hall effect sensor provides accurate
        RPM measurement.

        - 10-bit PWM control
        - 5 kHz PWM frequency
        - 100 Hz control loop
        - Magnetic pulse counting
        """)

with col2:
    with st.container(border=True):
        st.markdown("### 🔥 Temperature")
        st.markdown("""
        **Range**: 20 - 45 °C

        Temperature regulation using thermistor and bang-bang
        control with 39% power limit for safety.

        - Steinhart-Hart equation
        - ±0.5°C hysteresis
        - 12-bit ADC resolution
        - Sub-second response
        """)

with col3:
    with st.container(border=True):
        st.markdown("### 🧪 pH Level")
        st.markdown("""
        **Range**: 4.0 - 10.0 pH

        Dual pump system for precise pH control with deadband
        logic to prevent oscillation.

        - Moving average filter
        - Acid/Base dosing pumps
        - 3-second pulse dosing
        - 5-second cooldown
        """)

st.divider()

# Technical specifications
with st.expander("🔧 Technical Specifications", expanded=False):
    spec_col1, spec_col2 = st.columns(2)

    with spec_col1:
        st.markdown("""
        ### Hardware Components
        - **ESP32 DevKit-C V4**: Main controller
        - **ESP32 TTGO T-Display**: Gateway + LCD
        - **DC Motor**: Stirring mechanism
        - **Hall Sensor**: Speed measurement
        - **Thermistor**: Temperature sensing
        - **pH Sensor**: Analog pH measurement
        - **Peristaltic Pumps**: Acid/Base dosing
        - **Heating Element**: Temperature control

        ### Communication Protocols
        - **UART**: DevKit ↔ TTGO (115200 baud)
        - **MQTT**: TTGO ↔ Cloud (TLS port 8883)
        - **WiFi**: IEEE 802.11 b/g/n (WPA2 Enterprise)
        """)

    with spec_col2:
        st.markdown("""
        ### Software Stack
        - **Firmware**: Arduino Framework (ESP-IDF)
        - **Control**: Bang-bang with hysteresis
        - **MQTT Library**: PubSubClient (TLS enabled)
        - **Display**: TFT_eSPI library
        - **Dashboard**: Streamlit + Python
        - **Data Format**: JSON over MQTT

        ### Performance Metrics
        - **Control Loop**: 10 ms (100 Hz)
        - **UART Update**: 100 ms
        - **MQTT Publish**: 5 seconds
        - **Dashboard Refresh**: 1 second
        - **Total Latency**: < 6 seconds end-to-end
        """)

st.divider()

# Navigation guide
st.markdown("## 🗺️ Navigation Guide")

nav_col1, nav_col2, nav_col3, nav_col4 = st.columns(4)

with nav_col1:
    with st.container(border=True):
        st.markdown("### 💁 Introduction")
        st.markdown("View all three systems at once with real-time charts and system overview.")

with nav_col2:
    with st.container(border=True):
        st.markdown("### ⚙️ Stirring System")
        st.markdown("Control motor speed, view RPM trends, and analyze stirring performance.")

with nav_col3:
    with st.container(border=True):
        st.markdown("### 🔥 Heating System")
        st.markdown("Monitor temperature, adjust setpoints, and track thermal response.")

with nav_col4:
    with st.container(border=True):
        st.markdown("### 🧪 pH System")
        st.markdown("Regulate pH levels, observe pump activity, and maintain balance.")

st.divider()

# Footer
st.markdown("---")
st.markdown("""
<div style='text-align: center'>
    <p><strong>UCL Engineering Challenges 2024/2025</strong></p>
    <p>Group 5 - Bioreactor Control System</p>
    <p><em>Built with ESP32, MQTT, and Streamlit</em></p>
</div>
""", unsafe_allow_html=True)
