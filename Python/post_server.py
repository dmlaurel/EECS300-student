from bottle import run, request, post


@post('/')
def index():
    #data = request.body.read()
    data = request.json
    
    print(data.keys())
    print(data["counter"])
 
run(host='0.0.0.0', port=8090, debug=True)