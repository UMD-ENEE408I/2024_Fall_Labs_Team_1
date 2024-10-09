import threading

def processor(self):
    pass

def start_handler_thread():
    threading.Thread(name='Audio Processing Handler', args=(), target=processor).start()
