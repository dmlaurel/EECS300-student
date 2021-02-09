#get_request.py

import requests

# api-endpoint
URL = "http://192.168.1.174"

# location given here
data = "Picture"

# defining a params dict for the parameters to be sent to the API
PARAMS = {'action':'photo', 'table':"wood", 'person':'duncan'}

# sending get request and saving the response as response object
requests.get(url = URL, params = PARAMS)
#r =
# extracting data in json format
#data = r.json()
#


# extracting latitude, longitude and formatted address
# of the first matching location
#latitude = data['results'][0]['geometry']['location']['lat']