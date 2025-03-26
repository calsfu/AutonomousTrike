import overpy
import requests
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap

api = overpy.Overpass()

query = """
[out:json];
way["highway"]["name"="Main Street"](around:1000, 40.748817, -73.985428);
out body;
"""

result = api.query(query)

waypoints = []
for way in result.ways:
    for node in way.nodes:
        waypoints.append((node.lat, node.lon))

print(waypoints)

# routing

start = (42.3493, -71.1067)  # PHO
end = (42.353207, -71.118209)   # STUVI2

url = f'http://router.project-osrm.org/route/v1/driving/{start[1]},{start[0]};{end[1]},{end[0]}?overview=full&geometries=geojson'

# GET request
response = requests.get(url)
route_data = response.json()

route_coordinates = route_data['routes'][0]['geometry']['coordinates']

print(route_coordinates[:5])

# Create a Basemap instance
m = Basemap(projection='merc', llcrnrlat=42.34, urcrnrlat=42.36, llcrnrlon=-71.12, urcrnrlon=-71.10)

# Draw map features
m.drawcoastlines()
m.drawcountries()
m.drawstates()

# Convert coordinates to map projection (from lon, lat to x, y)
x, y = m([point[0] for point in route_coordinates], [point[1] for point in route_coordinates])

# Plot the route as a line
m.plot(x, y, color='blue', linewidth=2, marker='o', markersize=3)

# Mark start and end points
x_start, y_start = m(start[1], start[0])
x_end, y_end = m(end[1], end[0])

m.scatter(x_start, y_start, color='green', marker='D', label='Start')
m.scatter(x_end, y_end, color='red', marker='D', label='End')

# Add a legend
plt.legend()

# Show the plot
plt.show()