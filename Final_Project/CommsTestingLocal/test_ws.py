from time import sleep
import json
import datetime
from websocket import create_connection

ws = create_connection("ws://192.168.1.142:7000")

ws.send(json.dumps({'op': 'begin', 'name': 'BotKW'}))
ws.send(json.dumps({'op': 'message_transfer', 'name': 'BotKW', 'encrypted_message': '1FD1235816F44638', 'key': 'keynumb2enee408i'}))
#result =  ws.recv()

#print(f'Received {json.dumps(result)}')