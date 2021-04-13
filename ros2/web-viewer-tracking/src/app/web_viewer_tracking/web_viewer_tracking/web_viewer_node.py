from flask import Response
from flask import request
from flask import Flask
from flask import render_template
from flask import jsonify

# Topic names (use case specific):
# /camera0/image - just the image from the camera node
# /camera0/augmented_image - image + guidance system state?
# /cv/init_tracking - topic to tell CV initialize the tracker
# /cv/overlaid_image - topic for the CV to communicate image with overlay to anyone who cares

# Topic names (for generalized web viewer node)
# /web_viewer/in/display - topic for other nodes to publish an image for web viewer node to display
# /web_viewer/out/init_tracking - topic for other nodes to subscribe to if they want to know what part of the image should be tracked

app = Flask(__name__, template_folder='/webapp-templates') # TODO: make this a bit more dynamic?

@app.route("/")
def index():
    return jsonify({'success': True}), 200

def main():
    app.run('0.0.0.0', 8080, debug=True,
		threaded=True, use_reloader=False)

if __name__ == '__main__':
    main()
