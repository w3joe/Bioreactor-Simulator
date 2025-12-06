import streamlit as st
import pandas as pd
import json
from datetime import datetime
import mqtt_manager as mm
from mqtt_manager import initialize_session_state, start_mqtt_client

# Initialize MQTT
mm.initialize_session_state()
client = mm.start_mqtt_client()

# Page Configuration
st.set_page_config(
    page_title="Data Logging - Bioreactor Dashboard",
    page_icon="📊",
    layout="wide"
)

st.title("📊 Data Logging System")
st.markdown("---")

# Initialize logging session state
if 'data_log' not in st.session_state:
    st.session_state.data_log = []

if 'last_logged_time' not in st.session_state:
    st.session_state.last_logged_time = 0

# Auto-log new data from MQTT
@st.fragment(run_every=0.5)
def auto_log_data():
    """Automatically log new data points from MQTT"""
    current_time = datetime.now()

    # Check if we have new data from all three sensors
    has_rpm = "rpm_history_df" in st.session_state and not st.session_state.rpm_history_df.empty
    has_temp = "temp_history_df" in st.session_state and not st.session_state.temp_history_df.empty
    has_ph = "ph_history_df" in st.session_state and not st.session_state.ph_history_df.empty

    if has_rpm and has_temp and has_ph:
        # Get latest values
        rpm_df = st.session_state.rpm_history_df
        temp_df = st.session_state.temp_history_df
        ph_df = st.session_state.ph_history_df

        latest_rpm_time = rpm_df['time'].iloc[-1]

        # Only log if this is new data (not already logged)
        if latest_rpm_time > st.session_state.last_logged_time:
            log_entry = {
                'timestamp': current_time.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3],
                'unix_time': latest_rpm_time,
                'rpm': float(rpm_df['rpm'].iloc[-1]),
                'temperature': float(temp_df['temp'].iloc[-1]),
                'ph': float(ph_df['ph'].iloc[-1])
            }

            st.session_state.data_log.append(log_entry)
            st.session_state.last_logged_time = latest_rpm_time

            # Limit log size to last 10000 entries to prevent memory issues
            if len(st.session_state.data_log) > 10000:
                st.session_state.data_log = st.session_state.data_log[-10000:]

# Run auto-logging
auto_log_data()

# Display statistics
col1, col2, col3, col4 = st.columns(4)

with col1:
    st.metric("📝 Total Entries", len(st.session_state.data_log))

with col2:
    if st.session_state.data_log:
        first_time = st.session_state.data_log[0]['timestamp']
        st.metric("🕐 First Entry", first_time.split('.')[0])
    else:
        st.metric("🕐 First Entry", "N/A")

with col3:
    if st.session_state.data_log:
        last_time = st.session_state.data_log[-1]['timestamp']
        st.metric("🕐 Last Entry", last_time.split('.')[0])
    else:
        st.metric("🕐 Last Entry", "N/A")

with col4:
    if len(st.session_state.data_log) > 1:
        duration = (st.session_state.data_log[-1]['unix_time'] -
                   st.session_state.data_log[0]['unix_time'])
        minutes = int(duration // 60)
        seconds = int(duration % 60)
        st.metric("⏱️ Duration", f"{minutes}m {seconds}s")
    else:
        st.metric("⏱️ Duration", "N/A")

st.markdown("---")

# Control buttons
col1, col2, col3 = st.columns([1, 1, 2])

with col1:
    if st.button("🗑️ Clear All Logs", type="secondary", use_container_width=True):
        st.session_state.data_log = []
        st.session_state.last_logged_time = 0
        st.success("All logs cleared!")
        st.rerun()

with col2:
    if st.session_state.data_log:
        # Prepare JSON data for download
        json_data = json.dumps(st.session_state.data_log, indent=2)

        st.download_button(
            label="📥 Download as JSON",
            data=json_data,
            file_name=f"bioreactor_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json",
            mime="application/json",
            use_container_width=True
        )
    else:
        st.button("📥 Download as JSON", disabled=True, use_container_width=True)

with col3:
    st.info("💡 Data is automatically logged every time new MQTT data arrives")

st.markdown("---")

# Display recent logs in a table
st.subheader("📋 Recent Log Entries")

if st.session_state.data_log:
    # Convert to DataFrame for display
    df_display = pd.DataFrame(st.session_state.data_log)

    # Show only the last 100 entries in the table
    display_limit = min(100, len(df_display))
    df_recent = df_display.tail(display_limit).copy()

    # Format the display
    df_recent = df_recent[['timestamp', 'rpm', 'temperature', 'ph']]
    df_recent.columns = ['Timestamp', 'RPM', 'Temperature (°C)', 'pH']

    # Reverse to show most recent first
    df_recent = df_recent.iloc[::-1].reset_index(drop=True)

    # Display with custom styling
    st.dataframe(
        df_recent,
        use_container_width=True,
        height=400,
        column_config={
            "RPM": st.column_config.NumberColumn(
                "RPM",
                format="%.1f"
            ),
            "Temperature (°C)": st.column_config.NumberColumn(
                "Temperature (°C)",
                format="%.1f"
            ),
            "pH": st.column_config.NumberColumn(
                "pH",
                format="%.2f"
            )
        }
    )

    if len(st.session_state.data_log) > display_limit:
        st.caption(f"Showing last {display_limit} of {len(st.session_state.data_log)} total entries. Download JSON for complete data.")
else:
    st.info("⏳ No data logged yet. Data will be automatically logged when MQTT data is received.")

# Display storage information
st.markdown("---")
st.markdown("### 💾 Storage Information")

col1, col2 = st.columns(2)

with col1:
    st.markdown("""
    **Data Storage:**
    - Data is stored in browser session (Streamlit session state)
    - Automatically logs every new MQTT data point
    - Maximum 10,000 entries (oldest entries removed automatically)
    - Data persists during the current session
    - Cleared when browser tab is closed or page is refreshed
    """)

with col2:
    st.markdown("""
    **Logged Fields:**
    - `timestamp`: Human-readable date/time
    - `unix_time`: Unix timestamp from MQTT
    - `rpm`: Stirring motor speed
    - `temperature`: Measured temperature in °C
    - `ph`: Measured pH value
    """)

# Auto-refresh indicator
with st.container():
    st.caption("🔄 Auto-logging active - checking for new data every 0.5 seconds")
