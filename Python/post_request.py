import requests

url = "http://192.168.1.174/processData"

myobj = {'type': 'somevalue', 'name': 'bob'}

x = requests.post(url, json = myobj)


