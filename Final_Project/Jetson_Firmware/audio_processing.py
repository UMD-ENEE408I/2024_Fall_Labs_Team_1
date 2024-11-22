import threading
import queue
import logging
import pyaudio
from pydub import AudioSegment
import numpy as np
from scipy.io.wavfile import write, read
import scipy
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt
import wave
import speech_recognition as sr

import client_server

# Audio parameters
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 44100
CHUNK = 1024
duration = 5

recognizer = sr.Recognizer()

task_queue = queue.Queue()
def enqueue(message):
    task_queue.put(message)
    logging.info(f'Queued an audio request message: {message}')

def processor():
    while True:
        msg = task_queue.get(block=True, timeout=None)

        print('Opening Stream...')
        p = pyaudio.PyAudio()

        stream = p.open(format=FORMAT, channels=CHANNELS, rate=RATE, input=True, frames_per_buffer=CHUNK,
                        input_device_index=1)
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

        if msg['task'] == 'navigation':

            sound = AudioSegment.from_file("test.wav")
            print(f'Channels: {sound.channels}')

            split_sound = sound.split_to_mono()
            loudness = split_sound[0].rms

            print(f'Loudness: {loudness}')

            with sr.AudioFile('test.wav') as source:

                print("Listening...")
                audio = recognizer.listen(source)

            word = "NA"
            try:

                transcription = recognizer.recognize_google(audio)
                print(f"Transcription: {transcription}")

                if "left" in transcription.lower():
                    print("Detected LEFT")
                    word = "LEFT"
                elif "right" in transcription.lower():
                    print("Detected RIGHT")
                    word = "RIGHT"
            except sr.UnknownValueError:
                print("Could not understand the audio.")
            except sr.RequestError as e:
                print(f"Could not request results; {e}")

            client_server.send_to_client({'name': msg['name'], 'loudness': loudness, "word": word})

        elif msg['task'] == 'enc_chunk':

            print('Task enc chunk!\n reading...\n')

            rate, aud_data = read('test.wav')
            #rate, aud_data = 44000, np.random.random((9218368,))

            len_data = len(aud_data)
            N = rate * len_data

            print(f'Vars: {len_data}, {N} \n Now performing fft...\n')

            yf = fft(aud_data)

            print(f'FFT Yields: {abs(yf)}')

def start_handler_thread():
    threading.Thread(name='Audio Processing Handler', args=(), target=processor).start()
