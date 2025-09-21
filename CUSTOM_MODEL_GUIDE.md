-----

-----

# `CUSTOM_MODEL_GUIDE.md` (A guide for your repo)

## How to Use Your Own Custom AI Model

The "off-the-shelf" mode is great for testing, but the real power of this project comes from training a model on your *specific* objects. Here’s how to swap it in.

### 1\. Train and Convert Your Model

1.  **Train:** Use a tool like [Roboflow](https://roboflow.com/) to label your custom objects (e.g., "MyMedicineBottle", "MyTVRemote") and train a YOLO model (YOLOv8 is recommended).
2.  **Convert:** You will get a trained weights file (e.g., `best.pt`). You MUST convert this to the `.blob` format for the OAK-D.
      * Export your model to ONNX.
      * Use the [Luxonis BlobConverter](https://blobconverter.luxonis.com/) tool to convert the ONNX file into a `.blob` file.

### 2\. Place Your Model File

1.  In the `assistive_perception` package, create a new folder named `models`.
2.  Place your new file inside it. Your structure should look like this:
    ```
    assistive_perception/
    ├── models/
    │   └── my_custom_model.blob
    ├── assistive_perception/
    │   ├── __init__.py
    │   └── perception_node.py
    ...
    ```

### 3\. Modify `assistive_perception/perception_node.py`

Open `assistive_perception/assistive_perception/perception_node.py` and make these three changes:

**Change 1: Import `os` and `get_package_share_directory`**
At the top of the file, add these imports:

```python
import os
from ament_index_python.packages import get_package_share_directory
```

**Change 2: Update the Label Lists**
Find the `COCO_LABELS` and `TARGET_OBJECTS` lists near the top. Change them to match your new custom model. The order of your new label list **must** match the order you used during training.

  * **BEFORE:**
    ```python
    COCO_LABELS = ["person", "bicycle", ... , "bottle", "cup", ...]
    TARGET_OBJECTS = ["bottle", "cup", "cell phone", "remote"]
    ```
  * **AFTER:**
    ```python
    # These are YOUR custom object classes, in the exact order you trained them
    MY_LABELS = ["medicine_bottle", "tv_remote", "water_glass"]

    # These are the objects from your list that you want the robot to pick up
    TARGET_OBJECTS = ["medicine_bottle", "water_glass"]
    ```
    ...and inside the `timer_callback` function, update the label lookup:
  * **BEFORE:**
    ```python
    label = COCO_LABELS[detection.label]
    ```
  * **AFTER:**
    ```python
    label = MY_LABELS[detection.label]
    ```

**Change 3: Change the Model Path**
Find the `setup_pipeline` function. You need to change the line that loads the model from the "zoo" to load it from your new local file.

  * **BEFORE:**
    ```python
    self.get_logger().info("Loading 'yolov4_tiny_coco' from model zoo...")
    obj_nn = self.pipeline.create(dai.node.YoloDetectionNetwork)
    obj_nn.setConfidenceThreshold(0.5)
    obj_nn.setBlobPath(blobconverter.from_zoo(name="yolov4_tiny_coco", shaves=6))
    obj_nn.setNumClasses(80)
    ...
    ```
  * **AFTER:**
    ```python
    self.get_logger().info("Loading 'my_custom_model.blob' from local file...")

    # Get the absolute path to your model file
    model_path = os.path.join(
        get_package_share_directory('assistive_perception'),
        'models',
        'my_custom_model.blob'
    )

    obj_nn = self.pipeline.create(dai.node.YoloDetectionNetwork)
    obj_nn.setConfidenceThreshold(0.5)
    obj_nn.setBlobPath(model_path)
    obj_nn.setNumClasses(len(MY_LABELS)) # Automatically set num classes
    ...
    ```

**Final Note:** If your new model is a different architecture (e.g., YOLOv8 vs. YOLOv4), you may *also* need to update the other parameters like `obj_nn.setAnchors(...)` and `obj_nn.setAnchorMasks(...)` to match the new model's configuration.

Re-build your workspace (`colcon build`) and launch the project. It will now use your custom model\!