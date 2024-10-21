import threading
import queue
import logging

task_queue = queue.Queue()
def enqueue(message):
    task_queue.put(message)
    logging.info(f'Queued an audio request message: {message}')

def processor():
    while True:
        request = task_queue.get(block=True, timeout=None)

def start_handler_thread():
    threading.Thread(name='Audio Processing Handler', args=(), target=processor).start()
