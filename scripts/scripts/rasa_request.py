import requests
import json
API_ENDPOINT = "http://0.0.0.0:5005/webhooks/rest/webhook"
headers = { 'Content-type':'application/json'}
payload = '{"sender": "error", "message": "hello"}'

r = requests.post(API_ENDPOINT, data=payload.encode('utf-8'), headers=headers)
response = json.loads(r.content)
print(response[0]['text'])

