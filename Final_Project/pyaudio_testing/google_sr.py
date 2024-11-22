import pyaudio
import speech_recognition as sr


def process_audio():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    with mic as source:
        recognizer.adjust_for_ambient_noise(source, duration=2)
    print("Listening for the words 'left' or 'right'...")

    try:
        while True:
            with mic as source:
                print("Listening...")
                audio = recognizer.listen(source)

            try:

                transcription = recognizer.recognize_google(audio)
                print(f"Transcription: {transcription}")

                if "left" in transcription.lower():
                    print("Detected the word: LEFT")
                elif "right" in transcription.lower():
                    print("Detected the word: RIGHT")
            except sr.UnknownValueError:
                print("Could not understand the audio.")
            except sr.RequestError as e:
                print(f"Could not request results; {e}")
    except KeyboardInterrupt:
        print("\nExiting program.")


if __name__ == "__main__":
    process_audio()


