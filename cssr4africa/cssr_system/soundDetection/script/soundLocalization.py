import numpy as np
from scipy.signal import fftconvolve
from scipy.io import wavfile
import math

# Constants
SPEED_OF_SOUND = 346.0  # Speed of sound in air in m/s
DISTANCE_BETWEEN_EARS = 0.07  # Distance between microphones in meters
SAMPLING_RATE = 48000.0  # Sampling rate in Hz
INTENSITY_THRESHOLD = 400  # Intensity threshold for detecting sound

def gcc_phat(sig, refsig, fs=1, max_tau=None, interp=16):
    """
    Compute the Generalized Cross-Correlation - Phase Transform (GCC-PHAT)
    to estimate time delay between two signals.

    Parameters:
    - sig: Signal array
    - refsig: Reference signal array
    - fs: Sampling frequency
    - max_tau: Maximum time delay to consider
    - interp: Interpolation factor for the cross-correlation

    Returns:
    - tau: Estimated time delay
    - cc: Cross-correlation function
    """
    n = sig.size + refsig.size

    # FFT of the signals
    SIG = np.fft.rfft(sig, n=n)
    REFSIG = np.fft.rfft(refsig, n=n)

    # Cross-spectral density
    R = SIG * np.conj(REFSIG)

    # Apply PHAT weighting
    R /= np.abs(R)

    # Inverse FFT to get cross-correlation
    cc = np.fft.irfft(R, n=(interp * n))

    # Find the peak
    max_shift = int(interp * fs * max_tau)
    cc = np.concatenate((cc[-max_shift:], cc[:max_shift + 1]))

    shift = np.argmax(np.abs(cc)) - max_shift

    tau = shift / float(interp * fs)

    return tau, cc

def calculate_angle(itd):
    """
    Calculate the angle of arrival of the sound source based on ITD.

    Parameters:
    - itd: Interaural Time Difference

    Returns:
    - angle: Angle of arrival in degrees
    """
    z = itd * (SPEED_OF_SOUND / DISTANCE_BETWEEN_EARS)
    angle = math.asin(z) * (180.0 / np.pi)
    return angle

def process_audio(front_left, front_right):
    """
    Process audio data to estimate the sound source direction.

    Parameters:
    - front_left: Audio data from the front left microphone
    - front_right: Audio data from the front right microphone

    Returns:
    - angle: Estimated angle of arrival in degrees
    """
    # Calculate RMS values
    rms_front_left = np.sqrt(np.mean(np.square(front_left)))
    rms_front_right = np.sqrt(np.mean(np.square(front_right)))

    combined_intensity = rms_front_left + rms_front_right

    if combined_intensity > INTENSITY_THRESHOLD:
        # Compute ITD using GCC-PHAT
        itd, _ = gcc_phat(front_left, front_right, fs=SAMPLING_RATE, max_tau=DISTANCE_BETWEEN_EARS / SPEED_OF_SOUND)

        # Calculate angle based on ITD
        angle = calculate_angle(itd)
        return angle
    else:
        return None

def main():
    # Replace with actual audio data fetching method
    # For example, reading audio from a file or a microphone
    sampling_rate, audio_data = wavfile.read('stereo_audio.wav')
    
    # Assume front_left is left channel and front_right is right channel
    front_left = audio_data[:, 0]
    front_right = audio_data[:, 1]

    angle = process_audio(front_left, front_right)

    if angle is not None:
        print(f"Estimated sound direction angle: {angle:.2f} degrees")
    else:
        print("No significant sound detected.")

if __name__ == "__main__":
    main()