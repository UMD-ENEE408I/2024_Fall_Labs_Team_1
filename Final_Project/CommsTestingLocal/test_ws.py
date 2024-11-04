from time import sleep
import json
from websocket import create_connection

ws = create_connection("ws://172.20.80.1:7000")

ws.send(json.dumps({'op': 'begin', 'name': 'BotKW'}))

ws.send(json.dumps({'name': 'BotKW', 'op': 'image', 'model': 'color'}))

result =  ws.recv()
print(f'Received {json.dumps(result)}')