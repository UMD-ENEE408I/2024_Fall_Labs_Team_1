import threading
import time
import random
import json
import client_server, audio_processing, image_processing
import logging
logging.basicConfig(format='[%(threadName)-16.16s] %(levelname)s: %(message)s')
logging.getLogger().setLevel(logging.INFO)

def main():
    threading.Thread(name='ESP Server Start', target=client_server.start_server, daemon=True).start()
    threading.Thread(name='Audio Processing Start', target=audio_processing.start_handler_thread, daemon=True).start()
    threading.Thread(name='Image Processing Start', target=image_processing.start_handler_thread, daemon=True).start()

    # "Buzz 4 Times"
    encrypted_buzz4 = [
        'NQX++I0lFwe8KYXKdqQgwA==',
        '4nkDQPPTP++4OriyhCpDFQ==',
        'o5CXEvcluEPalm1WHVFsfg==',
        '/bzMNQ5rx5iGPRjycjRxgg==',
        'F1D4ZDdR3duuTZxvmwLhMw=='
    ]

    # "Buzz 5 Times"
    encrypted_buzz5 = [
        'd7h+ha 077EAFEM4tWnCdpQ==', # enee408ikeynumb1
        '8J+g1s3JSzhMj75Mhw8ezw==', # keynumb2enee408i
        'dnobomxgHTHQKDFGLSPAeA==', # keythreeenee408i
        'XWeGA/AfPeMqzQFBI5/OBQ==', # capstone8keyfour
        'IoxSIjHple91dN6ducBplQ=='  # thefifthkeyfor8i
    ]

    encrypted_messages = [encrypted_buzz4, encrypted_buzz5]

    buzz_count = random.randrange(0, len(encrypted_messages))
    encrypted_choice = random.randrange(0, 5)

    missing_block = encrypted_messages[buzz_count][encrypted_choice][0:8]

    logging.info(f'The Robot should buzz {buzz_count + 4} times!')
    logging.info(f'Encrypted message: {encrypted_messages[buzz_count][encrypted_choice]} encrypted with key {encrypted_choice}')
    logging.info(f'The Missing Block is: {missing_block}')

    msg_to_D = {
        'op': 'init_encrypt',
        'name': 'BotD',
        'encrypted': encrypted_messages[buzz_count][encrypted_choice][8:]
    }

    logging.info(f'Sending to Bot D: {msg_to_D}')

    # while True:
    #     res = client_server.send_to_client(json.loads(msg_to_D))
    #     if res == 0:
    #         break
    #     time.sleep(0.5)

    while True:
        try:
            time.sleep(1)
        except KeyboardInterrupt:
            break
    logging.info("Exiting")


if __name__ == '__main__':
    main()