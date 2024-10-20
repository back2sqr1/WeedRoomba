import streamlit as st
from inference_sdk import InferenceHTTPClient
import cv2
import numpy as np
from PIL import Image
import os

# Set up Roboflow inference clients for two models
API_KEY = "x1MbNYaNAwnOrzJlrpm1"  # Replace with your API key
MODEL_ID_1 = "weed-detection-5jm0z/4"  # Model 1 ID (Weed Detection)
MODEL_ID_2 = "plants-diseases-detection-and-classification/12"  # Model 2 ID (Plant Disease Detection)
CLIENT = InferenceHTTPClient(api_url="https://detect.roboflow.com", api_key=API_KEY)

# App title
st.title("WeedRoomba: Weed and Plant Disease Detection Scheme")

# Introduction
st.write("""
### Upload an image of your garden or agricultural land to detect:
- Weeds (Model 1)
- Plant Diseases (Model 2)
""")

# Function to load example images from 'examples/' folder
def load_example_images(folder_path="examples"):
    """Load images from the examples folder."""
    images = []
    if os.path.exists(folder_path):
        for file_name in os.listdir(folder_path):
            if file_name.endswith((".jpg", ".jpeg", ".png")):
                images.append(file_name)
    return images

# Sidebar for selecting example images or uploading a custom one
example_images = load_example_images()
selected_image = st.sidebar.selectbox("Choose an example image", ["None"] + example_images)
uploaded_file = st.file_uploader("Or upload a photo", type=["jpg", "jpeg", "png"])

# Determine the active image (uploaded or selected example)
if uploaded_file is not None:
    st.image(uploaded_file, caption="Uploaded Image", use_column_width=True)
    image = Image.open(uploaded_file)

elif selected_image != "None":
    image_path = os.path.join("examples", selected_image)
    image = Image.open(image_path)
    st.image(image, caption=f"Example Image: {selected_image}", use_column_width=True)

else:
    st.write("Please upload an image or select an example image to get predictions.")
    st.stop()  # Stop execution if no image is provided

# Convert the image to OpenCV format (for annotation)
image = np.array(image)
image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
cv2.imwrite("temp_image.jpg", image)  # Save temporarily for inference

# Run inference using Model 1 (Weed Detection)
st.write("Processing image and getting predictions from Model 1 (Weed Detection)...")
result_1 = CLIENT.infer("temp_image.jpg", model_id=MODEL_ID_1)

# Run inference using Model 2 (Plant Disease Detection)
st.write("Processing image and getting predictions from Model 2 (Plant Disease Detection)...")
result_2 = CLIENT.infer("temp_image.jpg", model_id=MODEL_ID_2)

# Annotate the image with predictions from both models
st.write("Annotating the image with predictions from both models...")

# Model 1 annotations (Weed Detection) - Green bounding boxes
if result_1.get("predictions"):
    st.write("Predictions from Model 1 (Weed Detection):")
    for prediction in result_1['predictions']:
        x, y = int(prediction['x']), int(prediction['y'])
        width, height = int(prediction['width']), int(prediction['height'])
        class_name = prediction['class']
        confidence = prediction['confidence']

        x_min = int(x - width / 2)
        y_min = int(y - height / 2)
        x_max = int(x + width / 2)
        y_max = int(y + height / 2)

        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        label = f"Weed: {class_name} ({confidence:.2f})"
        cv2.putText(image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# Model 2 annotations (Plant Disease Detection) - Red bounding boxes
if result_2.get("predictions"):
    st.write("Predictions from Model 2 (Plant Disease Detection):")
    for prediction in result_2['predictions']:
        x, y = int(prediction['x']), int(prediction['y'])
        width, height = int(prediction['width']), int(prediction['height'])
        class_name = prediction['class']
        confidence = prediction['confidence']

        x_min = int(x - width / 2)
        y_min = int(y - height / 2)
        x_max = int(x + width / 2)
        y_max = int(y + height / 2)

        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 0, 255), 2)
        label = f"Disease: {class_name} ({confidence:.2f})"
        cv2.putText(image, label, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

# Convert BGR back to RGB for display in Streamlit
annotated_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Display the final annotated image with results from both models
st.image(annotated_image, caption="Annotated Image with Detections from Both Models", use_column_width=True)
