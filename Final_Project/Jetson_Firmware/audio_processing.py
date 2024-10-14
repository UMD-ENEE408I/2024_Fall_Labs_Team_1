import threading
import logging

def processor():
    logging.info('Audio...')


def start_handler_thread():
    threading.Thread(name='Audio Processing Handler', args=(), target=processor).start()
