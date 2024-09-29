import pyaudio
import struct
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
 
CHUNK = 512
FORMAT = pyaudio.paInt16
CHANNELS = 1
DURATION = 5
 
p = pyaudio.PyAudio()

print("----------------------record device list---------------------")
info = p.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))

print("-------------------------------------------------------------")

RATE = int(p.get_device_info_by_index(23).get('defaultSampleRate'))
print(RATE)

stream = p.open(
	format = FORMAT,
	channels = CHANNELS,
	rate = RATE,
	input = True,
	frames_per_buffer = CHUNK,
	start = True,
	input_device_index = 23
	)
 
print("Recording...")

# Record audio data
frames = []
for i in range(0, int(RATE / CHUNK * DURATION)):
    data = stream.read(CHUNK)
    frames.append(np.frombuffer(data, dtype=np.int16))

print("Recording finished.")

stream.stop_stream()
stream.close()
p.terminate()

# Convert audio data to numpy array
audio_data = np.hstack(frames)

# Compute spectrogram
f, t, Sxx = signal.spectrogram(audio_data, RATE)

# Plot spectrogram
plt.pcolormesh(t, f, 10 * np.log10(Sxx), shading='gouraud')
plt.ylabel('Frequency [Hz]')
plt.xlabel('Time [sec]')
plt.title('Spectrogram')
plt.colorbar()
plt.show()



