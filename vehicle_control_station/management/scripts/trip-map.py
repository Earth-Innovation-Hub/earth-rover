import folium

def create_road_trip_map(itinerary, map_name):
    # Initialize a map centered around the first location
    road_trip_map = folium.Map(location=itinerary[0]['location'], zoom_start=6)

    # Plot each point in the itinerary
    for point in itinerary:
        folium.Marker(
            location=point['location'],
            popup=f"{point['city']}\nAttraction: {point['attraction']}\nDistance from previous: {point['distance']} miles",
            icon=folium.Icon(color='blue', icon='info-sign')
        ).add_to(road_trip_map)

        # Draw lines between points
        if 'prev_location' in point:
            folium.PolyLine([point['prev_location'], point['location']], color="blue", weight=2.5, opacity=1).add_to(road_trip_map)

    # Save the map to an HTML file
    road_trip_map.save(f"{map_name}.html")

# Itinerary for Houston to New Orleans
itinerary_new_orleans = [
    {'city': 'Houston, TX', 'location': [29.7604, -95.3698], 'attraction': 'Start Point', 'distance': '0'},
    {'city': 'Galveston, TX', 'location': [29.2998, -94.7946], 'attraction': 'Galveston Island Historic Pleasure Pier', 'distance': '50', 'prev_location': [29.7604, -95.3698]},
    # Add more stops here...
]

# Itinerary for Houston to Hot Springs
itinerary_hot_springs = [
    # Add stops for this route...
]

# Itinerary for Houston to San Antonio
itinerary_san_antonio = [
    # Add stops for this route...
]

# Create maps for each itinerary
create_road_trip_map(itinerary_new_orleans, "Houston_to_New_Orleans")
# create_road_trip_map(itinerary_hot_springs, "Houston_to_Hot_Springs")
# create_road_trip_map(itinerary_san_antonio, "Houston_to_San_Antonio")
