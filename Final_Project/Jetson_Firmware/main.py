import threading
import time
import client_server, audio_processing, image_processing
import logging
logging.basicConfig(format='[%(threadName)-16.16s] %(levelname)s: %(message)s')
logging.getLogger().setLevel(logging.INFO)

def main():
    threading.Thread(name='ESP Server Start', target=client_server.start_server, daemon=True).start()
    threading.Thread(name='Audio Processing Start', target=audio_processing.start_handler_thread, daemon=True).start()
    threading.Thread(name='Image Processing Start', target=image_processing.start_handler_thread, daemon=True).start()

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break
    logging.info("Exiting")


if __name__ == '__main__':
    main()