import json
import logging
import sys
import threading
import socket

import image_processing, audio_processing

from websocket_server import WebsocketServer

ws_server: WebsocketServer
plaintext_msg : str
msg_to_send_bot_d : dict

def new_client(client, server: WebsocketServer):
    logging.info(f"New ESP32 client connected and was given id {client['id']:d}.")

def client_left(client, _):
    logging.info(f"ESP32 client with id {client['id']:d} has disconnected.")

def message_received(client, server: WebsocketServer, message):

    try:
        msg_dict = json.loads(message)
        if msg_dict is None:
            logging.debug(f"{client['id']:d} sent an empty message.")
            return
    except json.JSONDecodeError:
        logging.debug(f'Invalid JSON: {message}')
        logging.debug(f"{client['id']:d} sent an invalid message.")
        return

    logging.info(f'Recieved Message: {msg_dict}')

    if msg_dict['op'] == 'begin':
        client['name'] = msg_dict['name']

        if msg_dict['name'] == 'BotD':
            logging.info(f'Sending to Bot D: {msg_to_send_bot_d}')
            send_to_client(msg_to_send_bot_d)

    elif msg_dict['op'] == 'image':
        image_processing.enqueue(msg_dict)
    elif msg_dict['op'] == 'audio':
        audio_processing.enqueue(msg_dict)
    elif msg_dict['op'] == 'message_transfer':
        if msg_dict['encrypted_message']:
            msg_to_D = {
                'op': 'encrypted_final',
                'name': 'BotD',
                'encrypted': msg_dict['encrypted_message'],
                'key': msg_dict['key']
            }
            send_to_client(msg_to_D)
        else:
            logging.info('No Encrypted Message Provided ')
    elif msg_dict['op'] == 'message_final':
        if msg_dict['plaintext']:
            logging.info(f'Recieved plaintext: {msg_dict['plaintext']}')
            if plaintext_msg == msg_dict['plaintext']:
                logging.info(f'Plaintext {msg_dict['plaintext']} is correct!')
            else:
                logging.info(f'Plaintext {msg_dict['plaintext']} is incorrect :(')

        else:
            logging.info('No Plaintext Message Provided ')

def get_ip_from_name(name_to_find):
    for client in ws_server.clients:
        if client['name'] == name_to_find:
            return client['address'][0]

def send_to_client(message):
    for client in ws_server.clients:
        if client['name'] == message['name']:
            ws_server.send_message(client, json.dumps(message))
            return 0
    logging.info(f'Couldn\'t find client {message['name']}')
    return 1

def start_server(final_msg, msg_to_d):
    global ws_server, plaintext_msg, msg_to_send_bot_d
    ws_server = None
    plaintext_msg = final_msg
    msg_to_send_bot_d = msg_to_d

    try:
        ws_server = WebsocketServer(host=socket.gethostbyname(socket.gethostname()), port=7000)
        #ws_server = WebsocketServer(host='127.0.0.1', port=7000)
    except OSError as e:
        logging.error(f'Issue starting client server {e.errno}.')
        exit(1)

    ws_server.set_fn_new_client(new_client)
    ws_server.set_fn_client_left(client_left)
    ws_server.set_fn_message_received(message_received)
    logging.info(f'Starting client ws_server @ IP {ws_server.host} on port {ws_server.port:d}.')
    threading.Thread(target=ws_server.run_forever, name='ESP32 WS Server', daemon=True).start()
 
