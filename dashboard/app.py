import queue
import streamlit as st


import numpy as np
import pandas as pd
import time
import requests
import mqtt_manager as mm 
from mqtt_manager import start_mqtt_client, safe_update_session_state

mm.initialize_session_state()
mqtt_client = mm.start_mqtt_client()

safe_update_session_state()

def get_latency():
    start_time = time.time()
    try:
        response = requests.get('https://www.google.com', timeout=5)
        latency  = (time.time()-start_time)*1000
        if response.status_code == 200:
            return True, latency
    except requests.exceptions.RequestException:
        pass

    return False, 0

def get_public_ip():
    try:
        response = requests.get('https://api.ipify.org?format=json', timeout=5)
        response.raise_for_status()  
        ip_data = response.json()
        return ip_data.get('ip', 'Unknown IP')
    
    except requests.exceptions.RequestException:
        return "Not Connected / API Error"

def get_wifi_status():
    connected, latency = get_latency()
    wifi_status = {
        'connected': connected,
        'latency': latency, 
        'ip': get_public_ip()}
    return wifi_status




#get a loader when first get in this site
initial_loader = st.empty()
if "initial_loader_done" not in st.session_state:
    with initial_loader.container():

        st.write('Your webapp is on the way......')

        progress = st.empty()
        bar = st.progress(0)

        for i in range(100):
            progress.text(f'{i+1}%')
            bar.progress(i + 1)
            time.sleep(0.05)

        time.sleep(0.5)
        st.success('...and now we are done!')
        time.sleep(1.5)

    initial_loader.empty()
    st.session_state.initial_loader_done=True


# Define the pages
main_page = st.Page("main_page.py", title="Welcome", icon="🏆")
page_1_introduction = st.Page("page_1_introduction.py", title="Introduction", icon="💁")
page_2_stirring_system = st.Page("page_2_stirring_system.py", title="Stirring System", icon="⚙️")
page_3_heating_system = st.Page("page_3_heating_system.py", title="Heating System", icon="🔥")
page_4_pH_system = st.Page("page_4_pH_system.py", title="pH System", icon="🧪")
page_5_data_logging = st.Page("page_5_data_logging.py", title="Data Logging", icon="📊")

# Set up navigation
pg = st.navigation([main_page, page_1_introduction, page_2_stirring_system, page_3_heating_system, page_4_pH_system, page_5_data_logging])

# Run the selected page
pg.run()




@st.fragment(run_every=1)
def show_network_status(target_container):
    is_connected, latency = get_latency()
    current_ip = get_public_ip()
    
    with target_container.container():
        if is_connected:
            st.markdown("**Status:** :green[Connected]🟢")
            st.metric("Latency", f"{latency:.0f} ms")
            st.markdown(f"**IP:** :blue[{current_ip}]")
        else:
            st.markdown("**Status:** :red[Disconnected]🔴")
            st.metric("Latency", "Timeout")
            st.markdown("**IP:** --")

@st.fragment(run_every=1) 
def show_mqtt_status(target_container):
    mm.safe_update_session_state()
    
    status_code, status_text = mm.get_mqtt_status(mqtt_client)
    
    target_container.markdown(f'**{status_text}**')

with st.sidebar:
    with st.container(border=True):
        st.markdown("### 🔗 MQTT Monitoring")
        mqtt_slot = st.empty()

    with st.container(border=True):
        st.markdown("### 🌐 Network Monitoring")
        network_slot = st.empty()

show_mqtt_status(mqtt_slot)
show_network_status(network_slot)
