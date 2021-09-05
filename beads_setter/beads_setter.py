#!/usr/bin/env python3
# Copyright 2020-2021 Naotaka Kawata
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
from std_msgs.msg import String
from flask import Flask, render_template, request
import os

app = Flask(__name__)
@app.route("/", methods=["GET","POST"])
def index():
    if request.method =="GET":
        return render_template('index.html')
    else:
        if str(request.form["text"])=="stop":
            os._exit(0)
        else:
            pub.publish(str(request.form["text"]))
        return render_template('index.html')
        
@app.route("/stop")
def stop():
    #bad approach
    os._exit(0)

if __name__ == "__main__":
    rospy.init_node('beads_map', anonymous=True)
    pub = rospy.Publisher('beads_positions', String, queue_size=100)
    r = rospy.Rate(100)
    app.run()
