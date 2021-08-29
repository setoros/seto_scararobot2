#!/usr/bin/env python
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

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from flask import Flask, render_template, request
import os
from rclpy.node import Node
from rclpy.qos import QoSProfile

static_folder_path = str(os.getenv('BS_DIR_PATH')) + '/static'
template_folder_path = str(os.getenv('BS_DIR_PATH')) + '/templates'
app = Flask(__name__, static_folder=static_folder_path, template_folder=template_folder_path)

rclpy.init()
node = Node('publisher')
qos_profile = QoSProfile(depth=10)
pub = node.create_publisher(String, 'beads_positions', qos_profile)

@app.route("/", methods=["GET","POST"])
def index():
    if request.method =="GET":
        return render_template('index.html')
    else:
        if str(request.form["text"])=="stop":
            os._exit(0)
        else:
            msg = String()
            msg.data = str(request.form["text"])
            pub.publish(msg)
        return render_template('index.html')          
        
def main():
    app.run()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
