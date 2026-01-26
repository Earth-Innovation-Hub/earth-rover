# django view to serve template html file
# Path: vehicle_control_station/views.py
from django.shortcuts import render

def ros_image_grid(request):
    return render(request, 'home.html', {})

