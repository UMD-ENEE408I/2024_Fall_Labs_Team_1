import json
import logging
import sys
import threading

from websocket_server import WebsocketServer

ws_server: WebsocketServer

def new_client(client, server: WebsocketServer):
    logging.debug(f"New ESP32 client connected and was given id {client['id']:d}.")

def client_left(client, _):
    logging.debug(f"ESP32 client with id {client['id']:d} has disconnected.")

def message_received(client, server: WebsocketServer, message):

    try:
        message = json.loads(message)
        if message is None:
            logging.debug(f'{(client['id'])} sent an empty message.')
            return
    except json.JSONDecodeError:
        logging.debug(f'Invalid JSON: {message}')
        logging.debug(f'{(client['id'])} sent an invalid message.')
        return

    if message['op'] == 'image':
        pass
    elif message['op'] == 'audio':
        pass

def start_server():
    global ws_server
    ws_server = None

    try:
        ws_server = WebsocketServer(host='192.168.100.100', port=7755)
    except OSError as e:
        logging.error(f'Issue starting client server {e.errno}.')
        exit(1)

    ws_server.set_fn_new_client(new_client)
    ws_server.set_fn_client_left(client_left)
    ws_server.set_fn_message_received(message_received)
    logging.debug(f'Starting client ws_server on port {ws_server.port:d}.')
    threading.Thread(target=ws_server.run_forever, name='ESP32 WS Server', daemon=True).start()
 
