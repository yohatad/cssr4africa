from flask import Flask, request, jsonify
from flask_cors import CORS
import random
import rospy
from naoqi_driver.srv import messageService

app = Flask(__name__)
CORS(app)  # Allow all origins


@app.route('/fetch-message', methods=['GET'])
def fetch_message():
    randomNumber = random.randint(1, 100)
    return jsonify({"text": randomNumber})

@app.route('/submit', methods=['POST'])
def receive_response():
    data = request.json
    user_name = data.get('name', 'Unknown')  # Default to 'Unknown' if name isn't provided
    user_message = data.get('response', '')
    print(f"Received message from {user_name}: {user_message}")
    
    rospy.wait_for_service('message_service')
    try:    
        send_message = rospy.ServiceProxy('message_service', messageService)
        # The user's message is now sent to the ROS service.
        ros_response = send_message(user_message)
        print(f"{user_name} submitted: {user_message}")
        response_message = f"Thank you, {user_name}, for your response."
        # You can include the ROS response in the Flask response if needed.
        return jsonify({"status": "Success", "message": response_message, "ros_response": ros_response.response_message})
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
        return jsonify({"error": "Service call failed"})

if __name__ == '__main__':
       # Initialize ROS node first.
    rospy.init_node('flask_ros_bridge', anonymous=False)

    # Then start Flask app. Make sure debug is False in production!
    app.run(debug=True, use_reloader=False)
