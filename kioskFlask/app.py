from flask import Flask, render_template

app = Flask(__name__)


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/burgers')
def burgers():
    return render_template('menu.html')


@app.route('/sides')
def sides():
    return render_template('side_menu.html')


@app.route('/drinks')
def drinks():
    return render_template('drink.html')


@app.route('/orderList')
def orderList():
    return render_template('order_list.html')


@app.route('/orderComplete')
def orderComplete():
    return render_template('order_completion.html')


@app.route('/voiceOrder')
def voiceOrder():
    return render_template('voiceOrder.html')


if __name__ == "__main__":
    app.run(host="127.0.0.1", port="5000", debug=True)
