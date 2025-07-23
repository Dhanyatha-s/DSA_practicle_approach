import streamlit as st
import osmnx as ox
import networkx as nx
import matplotlib.pyplot as plt
import heapq
from math import sqrt

st.set_page_config(layout="wide")
st.title("🚀 Fast A* Pathfinder on Real Map")

# ✅ Load map via place name instead of large bounding box
@st.cache_data(show_spinner="Downloading map... (Koramangala) 🚦")
def load_graph():
    return ox.graph.graph_from_place(
        "Koramangala, Bangalore, India",
        network_type="drive"
    )

G = load_graph()
st.success("✅ Map loaded: Koramangala, Bangalore")

# 📍 Input for Start & End Coordinates
st.markdown("### 📍 Enter Start & End Coordinates (within Koramangala)")
col1, col2 = st.columns(2)
with col1:
    start_lat = st.number_input("Start Latitude", value=12.944)
    start_lon = st.number_input("Start Longitude", value=77.610)
with col2:
    end_lat = st.number_input("End Latitude", value=12.936)
    end_lon = st.number_input("End Longitude", value=77.620)

# 📍 Convert lat/lon to nearest graph node
start_node = ox.distance.nearest_nodes(G, X=start_lon, Y=start_lat)
end_node = ox.distance.nearest_nodes(G, X=end_lon, Y=end_lat)

# ⚡ Heuristic function (Euclidean)
def euclidean(a, b):
    return sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

# 🚀 A* Search Algorithm
def a_star_search(G, start_node, end_node):
    g_score = {start_node: 0}
    came_from = {}
    open_set = [(0, start_node)]
    heapq.heapify(open_set)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == end_node:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start_node)
            return path[::-1]

        for neighbor in G.neighbors(current):
            dist = G[current][neighbor][0]['length']
            tentative_g = g_score[current] + dist

            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                h = euclidean(
                    (G.nodes[neighbor]['y'], G.nodes[neighbor]['x']),
                    (G.nodes[end_node]['y'], G.nodes[end_node]['x'])
                )
                heapq.heappush(open_set, (tentative_g + h, neighbor))

    return None

# 🔍 Find Path Button
if st.button("🔍 Find Route"):
    with st.spinner("Finding best route..."):
        path = a_star_search(G, start_node, end_node)

    if path:
        fig, ax = ox.plot_graph_route(G, path, route_color='red', node_size=0, show=False, close=False)
        st.success("✅ Path found!")
        st.pyplot(fig)
    else:
        st.error("❌ No path found. Try different coordinates.")
