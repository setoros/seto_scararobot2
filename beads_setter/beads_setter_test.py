from flask import Flask, render_template, request

app = Flask(__name__)
@app.route("/", methods=["GET","POST"])
def index():
    if request.method =="GET":
        return render_template('index.html')
    else:
        #if you want to send request.form["text"], use ROS node with int format
        return """
        {}!""".format(str(request.form["text"]))

if __name__ == "__main__":
    app.run()
    exit(0)
