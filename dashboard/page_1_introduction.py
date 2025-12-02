import streamlit as st
import numpy as np
import pandas as pd
import time


left_column, right_column = st.columns(2)
# You can use a column just like st.sidebar:
left_column.button('Press me!')

# Or even better, call Streamlit functions inside a "with" block:
with right_column:
    chosen = st.radio(
        'Sorting hat',
        ("Gryffindor", "Ravenclaw", "Hufflepuff", "Slytherin"))
    st.write(f"You are in {chosen} house!")


dataframe = pd.DataFrame(
    np.random.randn(10, 20),
    columns = ('col %d' % i for i in range (20)),
    #rows = ('row %d' % i for i in range(10))
    )

st.dataframe(dataframe.style.highlight_max(axis=0))


map_data = pd.DataFrame(
    np.random.randn(1000, 2) / [50, 50] + [37.76, -122.4],
    columns=['lat', 'lon'])

st.map(map_data)


x = st.slider('x')
st.write(x,'squared is', x*x)


st.text_input("Your name", key="name")
# You can access the value at any point with:
st.session_state.name


y = st.button('y')
st.write(y, 'squaredis ', y*y)


if st.checkbox('show dataframe'):
    chart_data = pd.DataFrame(
    np.random.randn(20,3),
    columns=['a','b','c'])
else: 
    chart_data = ''

chart_data


df = pd.DataFrame({
    'first column': [1, 2, 3, 4],
    'second column': [10, 20, 30, 40]
    })

option = st.selectbox(
    'Which number do you like best?',
     df['first column'])

'You selected: ', df['first column'][option]


'Starting a long computation...'

# Add a placeholder
latest_iteration = st.empty()
bar = st.progress(0)

for i in range(100):
  # Update the progress bar with each iteration.
  latest_iteration.text(f'Iteration {i+1}')
  bar.progress(i + 1)
  time.sleep(0.1)

'...and now were done!'

if "df" not in st.session_state:
    st.session_state.df = pd.DataFrame(np.random.randn(20, 2), columns=["x", "y"])

st.header("Choose a datapoint color")
color = st.color_picker("Color", "#FF0000")
st.divider()
st.scatter_chart(st.session_state.df, x="x", y="y", color=color)