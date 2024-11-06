
import pyaudio
from pydub import AudioSegment
import numpy as np
from scipy.io.wavfile import write
import wave


# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024
duration = 5

print('Opening Stream...')
p = pyaudio.PyAudio()

print("----------------------record device list---------------------")
info = p.get_host_api_info_by_index(0)
numdevices = info.get('deviceCount')
for i in range(0, numdevices):
        if (p.get_device_info_by_host_api_device_index(0, i).get('maxInputChannels')) > 0:
            print("Input Device id ", i, " - ", p.get_device_info_by_host_api_device_index(0, i).get('name'))

print("-------------------------------------------------------------")

stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK, input_device_index = 1)
frames = []

print('Recording for 5 seconds...')
for _ in range(int(RATE / CHUNK * duration)):
    data = stream.read(CHUNK)
    frames.append(data)

print('Done! Closing stream...')
stream.stop_stream()
stream.close()
p.terminate()

captured_audio = np.frombuffer(b''.join(frames), dtype=np.int16)
print(captured_audio.shape)

wf = wave.open('test.wav', 'wb')
wf.setnchannels(CHANNELS)
wf.setsampwidth(p.get_sample_size(FORMAT))
wf.setframerate(RATE)
wf.writeframes(b''.join(frames))
wf.close()

sound = AudioSegment.from_file("test.wav")
print(f'Channels: {sound.channels}')

split_sound = sound.split_to_mono()
loudness = split_sound[0].rms

print(f'Loudness: {loudness}')