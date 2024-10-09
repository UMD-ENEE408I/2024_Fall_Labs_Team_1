import logging
import threading
import time
import cv2

def enqueue(self, message):
    self.task_queue.put(message)

def processor(self):
    while True:
        request = self.task_queue.get(block=True, timeout=None)

def start_handler_thread():
    threading.Thread(name='Image Processing Queue Handler', args=(), target=processor).start()
