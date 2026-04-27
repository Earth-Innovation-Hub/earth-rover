import pygame
import pyaudio
import numpy as np
import pywt
import scipy.special as sp
import random

pygame.init()

# Set the window size and other display parameters
WINDOW_WIDTH, WINDOW_HEIGHT = 1920, 1000
WINDOW_TITLE = "Microphone Audio Waveform"
FPS = 24

# Initialize the Pygame window
window = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
pygame.display.set_caption(WINDOW_TITLE)
clock = pygame.time.Clock()

# Pyaudio settings
CHUNK_SIZE = 1920  # Number of audio samples per frame
SAMPLE_RATE = 44100  # Sampling rate (in Hz)

p = pyaudio.PyAudio()

# Open a microphone stream
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=SAMPLE_RATE,
                input=True,
                frames_per_buffer=CHUNK_SIZE)

running = True
# background_image_path = '/home/dreamslab/rainbow-colors-background.jpg'  # Change this to your image path
# background_image_path = '/home/dreamslab/rainbow-2.jpeg'  # Change this to your image path
background_image_path = '/home/jdas/ros2_ws/src/tutorial_interfaces/src/milkyway-pano.jpg'  # Change this to your image path
background_image = pygame.image.load(background_image_path).convert_alpha()
background_image = pygame.transform.scale(background_image,
                                          (WINDOW_WIDTH, WINDOW_HEIGHT))  # Scale the image to fit the window

# Optional: Set a specific transparency level (0 fully transparent, 255 fully opaque)
alpha_value = 150
background_image.fill((255, 255, 255, alpha_value), None, pygame.BLEND_RGBA_MULT)


def compute_fft(audio_data):
    # Compute the FFT of the audio data
    spectrum = np.fft.fft(audio_data)
    frequencies = np.fft.fftfreq(len(spectrum)) * SAMPLE_RATE

    # Take the absolute value and scale it to fit in the window height
    spectrum = np.abs(spectrum)
    spectrum = spectrum / np.max(spectrum)
    spectrum = spectrum * (WINDOW_HEIGHT / 2)

    return frequencies, spectrum


def compute_wavelet_transform(audio_data):
    # Compute the wavelet transform of the audio data using 'db1' wavelet
    coeffs = pywt.dwt(audio_data, 'db1')
    cA, cD = coeffs

    # Normalize the wavelet transform coefficients to fit in the window height
    cA = cA / np.max(np.abs(cA))
    cA = cA * (WINDOW_HEIGHT / 2)

    cD = cD / np.max(np.abs(cD))
    cD = cD * (WINDOW_HEIGHT / 2)

    return cA, cD


def compute_chebyshev_polynomials(audio_data, num_polynomials=5):
    # Normalize audio data to fit in the window height
    audio_data_normalized = audio_data / 32767.0
    audio_data_normalized = audio_data_normalized * (WINDOW_HEIGHT / 2)

    # Calculate Chebyshev polynomials up to the specified order
    polynomials = []
    for i in range(num_polynomials):
        polynomial = sp.chebyt(i)
        polynomial_vals = polynomial(audio_data_normalized)
        polynomials.append(polynomial_vals)

    return polynomials


def normalize_chebyshev_polynomials(audio_data, num_polynomials=5):
    # Normalize audio data to fit in the window height
    audio_data_normalized = audio_data / 32767.0
    audio_data_normalized = audio_data_normalized * (WINDOW_HEIGHT / 2)

    # Calculate Chebyshev polynomials up to the specified order
    polynomials = []
    for i in range(num_polynomials):
        polynomial = sp.chebyt(i)
        polynomial_vals = polynomial(audio_data_normalized)

        # Normalize the polynomial values to the range [-1, 1]
        polynomial_vals = 250 * (polynomial_vals - np.min(polynomial_vals)) / (
                    np.max(polynomial_vals) - np.min(polynomial_vals)) - 1
        polynomials.append(polynomial_vals)

    return polynomials


def draw_scatter(surface, cA, cD):
    """ Draw a scatter plot of cA vs cD """
    for i in range(len(cA)):
        x = int(cA[i] + WINDOW_WIDTH // 2)  # Shift the x-coordinate to the right
        y = int(cD[i] + WINDOW_HEIGHT // 2)  # Shift the y-coordinate to the center
        pygame.draw.circle(surface, (255, 255, 255), (x, y), 2)


while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Read audio data from the microphone stream
    audio_data = np.frombuffer(stream.read(CHUNK_SIZE), dtype=np.int16)

    # Normalize audio data to fit in the window height
    audio_data_normalized = audio_data / 32767.0
    waveform = np.int16(audio_data_normalized * (WINDOW_HEIGHT / 2) + WINDOW_HEIGHT / 2)
    frequencies, spectrum = compute_fft(audio_data_normalized)
    cA, cD = compute_wavelet_transform(audio_data_normalized)

    window.blit(background_image, (0, 0))
    # window.fill((0, 0, 0))

    # Draw the waveform
    for i in range(1, len(waveform)):
        pygame.draw.aaline(window, (250, 250, 255), (i - 1, waveform[i - 1]), (i, waveform[i]), 4)

    # Draw the FFT spectra
    for i in range(1, len(frequencies)):
        pygame.draw.aaline(window, (255, 0, 100), (i - 1, WINDOW_HEIGHT - spectrum[i - 1]),
                           (i, WINDOW_HEIGHT - spectrum[i]), 4)

    # Compute and draw the Chebyshev polynomials (bottom part of the window)
    chebyshev_polynomials = normalize_chebyshev_polynomials(audio_data)

    for i, polynomial_vals in enumerate(chebyshev_polynomials):
        color = pygame.Color(random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        poly_vals = len(polynomial_vals)
        # print(len(chebyshev_polynomials), poly_vals)
        for j in range(1, poly_vals):
            pygame.draw.aaline(window, color, (j - 1, polynomial_vals[j - 1]),
                               (j, polynomial_vals[j]), 4)

    # Draw the cA vs cD scatter plot
    draw_scatter(window, cA, cD)

    pygame.display.flip()
    clock.tick(FPS)

# Clean up
stream.stop_stream()
stream.close()
p.terminate()
pygame.quit()
