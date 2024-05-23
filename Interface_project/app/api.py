from flask import Flask, render_template
import requests
import json
import schedule
import time 
import threading

app = Flask(__name__)
result = None
result_img = None

def call_api():
    global result
    url ='http://127.0.0.1:8042/restgatewaydemo/getmultcoords'
    data ={}
    headers = {'Content-type': 'application/json'}

    response = requests.post(url, data=json.dumps(data), headers=headers)
    result = response.json()
    # result["values"].pop()
    with open('result.json', 'w') as f:
        json.dump(result, f)

def call_api_image():
    global result_img
    url ='http://127.0.0.1:8042/restgatewaydemo/getimageresult'
    data ={}
    headers = {'Content-type': 'application/json'}

    response = requests.post(url, data=json.dumps(data), headers=headers)
    result_img = response.json()
    with open('result_img.json', 'w') as f:
        json.dump(result_img["data"], f)


def schedule_api_call():
    while True:
        schedule.run_pending()
        time.sleep(1)


if __name__ =='__main__':
    call_api()
    schedule.every(0.1).seconds.do(call_api)
    schedule.every(0.1).seconds.do(call_api_image)

    t = threading.Thread(target=schedule_api_call)
    t.start()

    @app.route('/api/result/x')
    def get_result_x():
        return str(result["x"])

    @app.route('/api/result/y')
    def get_result_y():
        return str(result["y"])

    @app.route('/api/result/theta')
    def get_result_theta():
        return str(result["theta"])

    @app.route('/api/image')
    def get_image():
        if result_img is not None:
            return "data:image/jpg;base64," + result_img["data"].replace('"','')
        else:
            return "data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAACwAAAAwCAYAAABqkJjhAAAAAXNSR0IArs4c6QAAAARnQU1BAACxjwv8YQUAAAAJcEhZcwAAEnQAABJ0Ad5mH3gAAAcRSURBVGhD1VlbbxvHGT2zN3JJiqRiW5bjSxI7jtMEDdDEfelDUfSp6C8qULRAgPyBIAj60KQPfSmK9D80TwGCwEhaoEZi2DUsX6JEFimKXu59v55vlxIkUjSUmjKZA49ndzScOfvtd501QuBHgDiOy/5HQ1hpWpYFa3w/d+gG85JFlmUwxpTXx5awTirYbP1fb4xVdbzP+dzluLbREyBMgX4fGA6BbpdtFXAdoG5DHL+c75TzLV5zTV1or9lsB6D0fN8vSe8NHA8ZWyISsgskLa93k7wa//vHMrr8lgxMQ0LjSWFsCVqu7NYhmU2ZNFYk9ZqSwJKHF14Tef9DkSiRdMRF+HtdT9u2LppzzQkoTW3tdrt8bcdCoItx8ShOZUvXHG7KNxfPy8CHRI4joQcpHEjgkSActrqkVp29J8IHKCwjO5yrDxM5fKAa5P5P3hCJvxfp5RXPuHr+oxBFUUV8fH8MUKr8J3fviVy9IiP+eLPLzY0vKYkqaUGnJJtZjsQkpeSUrN6ndtUCVx+iIRma5UPc4W/z8y+JPLgtg/L9zQb1eDbhWPR1U6raa9NL2RRpdktS82x7Uv/0+hWJdBvyflL2uvdhzDY6oTHRsOyEVuoB0c4NFOu/QbtGf7gbjifNB7qPRWMznsu179IWzwDcN6kB3PoQnuolIraCVt+QHgarb8KxAmR0hJ1wwpSfEUKPFdsWCitBaNo4tf0l8ubLQEqB8RkOYsoP53Q3Mm6KxqO7CFZexUoaoBAXrXi+ZBXUTNTzHH7qYTXdRbL6Buyb/yLZikMV4+jWeDslYSVqYvpHvg472sawdRENhLBz+tHngIFno5PHSJtduPf+AzRfVGdPMgFGpjktYVNY6Ds9/PvXv0W2fhWeofcsVJNOLCgeQifNkHM/tygQXLqMz37+NmKHos2aFCalrBI+BDXMm59L5O7508W2R7QgufMFHQeJ8d8RRhcgOXMdpv813OekBk/DEyeHfeEiarfv8R3zbY/H97H1179Qy7+GyScdymLQoAJbDzZgffAnhBHta1LCO2vX0I1uoR+vYTXpjUcXh8SpwysiPDjfRmNj+wg/7NM9xDn9IlDLqpRukRArR2psMNeAv/twmnBq8xUw+WMWUHn0RYNOeujaWGEUvHH6lSMIOwaJ7dGRp/S98w8SPxQJKajx7zLu+usXjlCJbgv5kEFZbEq5ijSLRJlnsNewnTDbnyJcTmAzJFzVGIuFkEsF6rJorTIB26mh0PrJaPGyPFC5UlsPE86ZgKhQNSNbBukehX3CBWO3gtUD3dlyOIijcEjCtm3DzZi466gc+tPSYJ+VHlIo0iKhL5ax0S0PLAqzoHuYEqPrukwxXarEkugw33TOOm3AuDBYOYJwkaVlz+pkacCErUziW40z04Rjz0ItL+AtiVcrLApQ6rB94O7q5enAUTBup3wOj5ZnliHS2RkL+BUkMkSj93hawlG7UU4o6+wlQEF7sq0hhitnEHS604St995ltdqHMa3xyGLhUndTx8XZP/wOzTCbVgk9i6hdWkdyfwe18mxxsRiZDNnLryD/7y3YYTgt4TrV1vzzEzxGXAUQTqn65wNWmeOrat8t1hPtv32EbuigCW+asLqPby+/hW/e+RnihoPAK2AdegcnDYOMgYu5GYasNm5duQZc/xVCeonMZ4yYVAkW/9Uhs+kh9a/BRI+ZJdkctp+L18gsVjt8u498wYtBh0njQ+xaTbQ1PFgUXjXtAChhVkkk/QKS8Hs4X9xASD3JTYLYYaHtzF/cqgbaRq6D0Ilgzr2E03/+B/PfHZIkWZ2kZ2y2utpJCR+APpQbU6qyAZy7jp1gG53MOwFJUxBMXTzmMT05hRfir5g3dLBTa1FslN4BzCbM0YRz9TyOk7jmfYb1txGaHTTmnHtqVRHRf+l5dT34jgMtDBwHnYSbM/IexEz7z5n8eFGBRGcEOXrmEgdvMkTWENsON+H4vkWPMXk/if2/W9Xvx0hs6maS4bs3f8q9VuhaDXwWEDJBVvFUlZiCfsgZPAB++QuMbt5HXnPgJz7DZ1wm/RF1XPOQWVCT0SxwqwmsPXFUESBtQT8XnHp4B2FQh39ufTx7BpTwcRDpIX5WfT0KdGB3U75cOy2xxyUcI48odLrK8vhf45E2vdam6QvLR2GmKMMu59DpDFuW3L7yuki/J4+5bFzuIVKUp5GzcWzCUuRclHZbfpkhea4+iLNypzvv/VF6tfOyqc/v2PtNP7poyw3HbUttXganr8rGu7/nj7nMIGTH9Qpdk0TZz/qKtIcfphLHQTyqvoFsUHX6W8DZNXqYs0CTuYlPXXhGzJ8wVThKEhj6VJt+U6EpbXk18b3i/8EJEKZ09fCD0bFMBrTUKmO7GiMTg2fEiUi49PVsSelWWAyofHW8EvgzAPgfcGwCRRWwLHgAAAAASUVORK5CYII=" #cambiar esta line
        
    @app.route('/')
    def show_result():
        return render_template('puzzlebot.html')
    
    app.run(debug=True, port=8002)



