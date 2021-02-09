from bottle import run, request, post
 
@post('/')
def index():
    data = request.body.read()
    print(data) 
 
run(host='0.0.0.0', port=8090, debug=True)