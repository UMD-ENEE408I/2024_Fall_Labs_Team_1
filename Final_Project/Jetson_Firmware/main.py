import logging
logging.basicConfig(format='[%(threadName)-16.16s] %(levelname)s: %(message)s', level=logging.DEBUG)

import threading

import client_server, audio_processing, image_processing

def main():
    threading.Thread(name='ESP Server Start', target=client_server.start_server, daemon=True).start()
    threading.Thread(name='Audio Processing Start', target=audio_processing.start_server, daemon=True).start()
    threading.Thread(name='Image Processing Start', target=image_processing.start_server, daemon=True).start()
