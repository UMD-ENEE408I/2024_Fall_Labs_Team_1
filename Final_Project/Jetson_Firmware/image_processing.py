import logging
import threading
import time
import cv2
import queue

task_queue = queue.Queue()
def enqueue(message):
    task_queue.put(message)

def processor():
    while True:
        request = task_queue.get(block=True, timeout=None)

def start_handler_thread():
    threading.Thread(name='Image Processing Queue Handler', args=(), target=processor).start()
