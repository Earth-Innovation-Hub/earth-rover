"""Vehicle Control Station URL configuration."""
from django.urls import path

from .mission_views import mission_page, mission_status
from .views import home, recording_start, recording_status, recording_stop, ros_nodes, system_info

urlpatterns = [
    path('', home, name='home'),
    path('ros-image/', home, name='ros_image_grid'),
    path('mission/', mission_page, name='mission_page'),
    path('api/system-info/', system_info, name='system_info'),
    path('api/recording/status/', recording_status, name='recording_status'),
    path('api/recording/start/', recording_start, name='recording_start'),
    path('api/recording/stop/', recording_stop, name='recording_stop'),
    path('api/ros-nodes/', ros_nodes, name='ros_nodes'),
    path('api/mission/', mission_status, name='mission_status'),
]
