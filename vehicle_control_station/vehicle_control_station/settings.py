"""Django settings for the Earth Rover Vehicle Control Station.

Runs on the rover as er-vcs.service (scripts/startup/mission.yaml). Values come
from the environment; the unit reads ~/.config/earth-rover/secrets.env.
"""

from pathlib import Path
import os
import sys

BASE_DIR = Path(__file__).resolve().parent.parent

SECRET_KEY = os.getenv('SECRET_KEY')
if not SECRET_KEY:
    sys.exit("SECRET_KEY is not set (put it in ~/.config/earth-rover/secrets.env).")

DEBUG = os.getenv('VCS_DEBUG', '').lower() in ('1', 'true', 'yes')

# The console is reachable only from loopback and the cable network (ufw), so
# any Host header is accepted unless VCS_ALLOWED_HOSTS narrows it.
ALLOWED_HOSTS = [h.strip() for h in os.getenv('VCS_ALLOWED_HOSTS', '*').split(',') if h.strip()]

INSTALLED_APPS = [
    'django.contrib.contenttypes',
    'django.contrib.staticfiles',
]

MIDDLEWARE = [
    'django.middleware.security.SecurityMiddleware',
    'whitenoise.middleware.WhiteNoiseMiddleware',
    'django.middleware.common.CommonMiddleware',
    'django.middleware.csrf.CsrfViewMiddleware',
    'django.middleware.clickjacking.XFrameOptionsMiddleware',
]

ROOT_URLCONF = 'vehicle_control_station.urls'

TEMPLATES = [
    {
        'BACKEND': 'django.template.backends.django.DjangoTemplates',
        'DIRS': [BASE_DIR / 'templates'],
        'APP_DIRS': True,
        'OPTIONS': {
            'context_processors': [
                'django.template.context_processors.request',
            ],
        },
    },
]

WSGI_APPLICATION = 'vehicle_control_station.wsgi.application'

# No models are used; nothing reads or writes the database.
DATABASES = {
    'default': {
        'ENGINE': 'django.db.backends.sqlite3',
        'NAME': BASE_DIR / 'db.sqlite3',
    }
}

LANGUAGE_CODE = 'en-us'
TIME_ZONE = 'UTC'
USE_I18N = False
USE_TZ = True

X_FRAME_OPTIONS = 'SAMEORIGIN'
# Tile servers (OpenStreetMap in particular) refuse browser requests without a Referer;
# Django's default 'same-origin' strips it from cross-site tile fetches.
SECURE_REFERRER_POLICY = 'strict-origin-when-cross-origin'

STATIC_URL = '/static/'
STATICFILES_DIRS = [BASE_DIR / 'vehicle_control_station' / 'static']
# Served straight from STATICFILES_DIRS by WhiteNoise; no collectstatic step.
STATIC_ROOT = None
WHITENOISE_USE_FINDERS = True
WHITENOISE_AUTOREFRESH = DEBUG

DEFAULT_AUTO_FIELD = 'django.db.models.BigAutoField'

# Overrides for the browser's rosbridge / web_video_server URLs; by default the
# page uses the host it was loaded from (views.home).
ROS_BRIDGE_URL = os.getenv('ROS_BRIDGE_URL', '')
VIDEO_SERVER_URL = os.getenv('VIDEO_SERVER_URL', '')
ROS_BRIDGE_PORT = os.getenv('ROS_BRIDGE_PORT', '9090')
VIDEO_SERVER_PORT = os.getenv('VIDEO_SERVER_PORT', '8080')
